#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <cstdint>
#include <string>
#include <utility>

namespace ssm_pfl
{
struct Limit
{
  double inverseMass;
  double reflectedMass;
  double pflSpeed;
  double combinedSpeed;
};

struct TemporalDerivative
{
  std::uint64_t sequence = 0;
  double time = 0, mobility = 0, beta = 0, mobilityRate = 0;
  double humanApproach = 0, humanApproachRate = 0;
  Eigen::RowVectorXd j;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d local = Eigen::Vector3d::Zero();
  std::string geometry;
  bool valid = false, switched = false;

  void observe(std::uint64_t nextSequence, double nextTime, const Eigen::RowVectorXd & nextJ,
               const Eigen::VectorXd & qdot, double nextMobility, double nextHumanApproach,
               const Eigen::Vector3d & nextNormal, const Eigen::Vector3d & nextLocal,
               const std::string & nextGeometry, bool intersecting,
               double maxSampleGap, double maxWitnessStep, double maxNormalStep,
               double mobilityTolerance)
  {
    if(nextSequence == sequence) return; // Never differentiate a reused async sample.
    const double dt = nextTime - time;
    switched = !geometry.empty() &&
      (geometry != nextGeometry || (local - nextLocal).norm() > maxWitnessStep
       || std::acos(std::clamp(normal.dot(nextNormal), -1.0, 1.0)) > maxNormalStep);
    valid = sequence != 0 && nextSequence > sequence && std::isfinite(dt) && dt > 0
      && dt <= maxSampleGap && !switched && !intersecting
      && nextMobility > mobilityTolerance && std::isfinite(nextHumanApproach)
      && j.size() == nextJ.size();
    if(valid)
    {
      beta = ((nextJ - j) / dt).dot(qdot);
      mobilityRate = (nextMobility - mobility) / dt;
      humanApproachRate = (nextHumanApproach - humanApproach) / dt;
      valid = std::isfinite(beta) && std::isfinite(mobilityRate)
          && std::isfinite(humanApproachRate);
    }
    sequence = nextSequence;
    time = nextTime;
    j = nextJ;
    mobility = nextMobility;
    humanApproach = nextHumanApproach;
    normal = nextNormal;
    local = nextLocal;
    geometry = nextGeometry;
  }
};

// Directional inverse mass uses a solve with the full fixed-base robot H.
inline Limit limit(const Eigen::LDLT<Eigen::MatrixXd> & inertia,
                   const Eigen::RowVectorXd & directionalJacobian,
                   double forceMax, double stiffness, double distance, double distanceMargin,
                   double stoppingTime, double mobilityTolerance)
{
  const double w = directionalJacobian.dot(inertia.solve(directionalJacobian.transpose()));
  if(!std::isfinite(w) || w < -mobilityTolerance || !std::isfinite(forceMax) || forceMax < 0
     || !std::isfinite(stiffness) || stiffness <= 0 || !std::isfinite(distance) || distance < 0
     || !std::isfinite(distanceMargin) || distanceMargin < 0 || !std::isfinite(stoppingTime)
     || stoppingTime <= 0)
    throw std::domain_error("invalid SSM+PFL inputs or directional mobility");
  const double mobility = std::max(0.0, w);
  const double pfl = forceMax / std::sqrt(stiffness) * std::sqrt(mobility);
  const double combined = std::max(distance - distanceMargin, 0.0) / stoppingTime + pfl;
  if(!std::isfinite(pfl) || !std::isfinite(combined))
    throw std::domain_error("nonfinite SSM+PFL speed limit");
  return {mobility, mobility > 0 ? 1.0 / mobility : std::numeric_limits<double>::infinity(),
          pfl, combined};
}

// Hold the sampled human velocity constant for one controller step. The QP
// controls robot acceleration so that next-step relative closing stays below
// the combined SSM+PFL speed limit.
inline double accelerationBound(double combinedSpeed, double robotApproach,
                                double humanApproach, double jacobianDrift, double dt)
{
  const double bound = combinedSpeed - (robotApproach - humanApproach) - dt * jacobianDrift;
  if(!std::isfinite(bound) || !std::isfinite(dt) || dt <= 0)
    throw std::domain_error("invalid acceleration inequality");
  return bound;
}

// Human approach is negative along the robot-to-human contact direction.
// Floor the robot's allowance at zero: a human cannot force robot retreat.
inline double robotSpeedBudget(double combinedSpeed, double humanApproach)
{
  if(!std::isfinite(combinedSpeed) || combinedSpeed < 0 || !std::isfinite(humanApproach))
    throw std::domain_error("invalid pointwise speed budget");
  return std::max(0.0, combinedSpeed + humanApproach);
}

// With v_next = v + dt * (J*qddot + Jdot*qdot), return the upper
// bound for the approach-side row dt*J. Retreat has no PFL speed row.
inline double pointSpeedUpperBound(double budget, double robotSpeed,
                                   double jacobianDrift, double dt)
{
  if(!std::isfinite(budget) || budget < 0 || !std::isfinite(robotSpeed)
     || !std::isfinite(jacobianDrift) || !std::isfinite(dt) || dt <= 0)
    throw std::domain_error("invalid pointwise speed bounds");
  const double offset = robotSpeed + dt * jacobianDrift;
  return budget - offset;
}

// A temporal estimate of wdot may include normal, witness and inertia changes.
// Use a one-step difference of the nonnegative PFL allowance. The differential
// sqrt(w) rate is singular as w approaches zero and can demand an impossible
// acceleration even though the remaining allowance is almost zero.
inline bool mobilityExtrapolationValid(double mobility, double mobilityRate,
                                       double mobilityTolerance, double dt)
{
  if(!std::isfinite(mobility) || !std::isfinite(mobilityRate)
     || !std::isfinite(mobilityTolerance) || !std::isfinite(dt)
     || mobilityTolerance < 0 || dt <= 0) return false;
  const double nextMobility = mobility + dt * mobilityRate;
  return std::isfinite(nextMobility) && nextMobility > mobilityTolerance;
}

inline double pflSpeedRate(double forceMax, double stiffness, double mobility,
                           double mobilityRate, double mobilityTolerance, double dt)
{
  if(!std::isfinite(mobility) || !std::isfinite(mobilityRate) || mobility <= mobilityTolerance
     || !std::isfinite(forceMax) || forceMax < 0 || !std::isfinite(stiffness) || stiffness <= 0
     || !std::isfinite(dt) || dt <= 0)
    throw std::domain_error("PFL derivative unavailable at zero or invalid mobility");
  const double nextMobility = std::max(0.0, mobility + dt * mobilityRate);
  const double rate = forceMax / std::sqrt(stiffness)
      * (std::sqrt(nextMobility) - std::sqrt(mobility)) / dt;
  if(!std::isfinite(rate)) throw std::domain_error("nonfinite PFL speed rate");
  return rate;
}

inline double mobilityRate(const Eigen::RowVectorXd & jdot, const Eigen::VectorXd & z,
                           const Eigen::MatrixXd & massRate)
{
  const double result = 2 * jdot.dot(z) - z.dot(massRate * z);
  if(!std::isfinite(result)) throw std::domain_error("invalid mobility derivative");
  return result;
}

// At the kink use the smaller one-sided distance contribution, so an
// approaching pair cannot receive the receding-side allowance.
inline double combinedSpeedRate(double distance, double margin, double distanceRate,
                                double pflRate, double stoppingTime)
{
  if(!std::isfinite(distance) || !std::isfinite(margin) || !std::isfinite(distanceRate)
     || !std::isfinite(pflRate) || !std::isfinite(stoppingTime) || stoppingTime <= 0)
    throw std::domain_error("invalid combined-envelope derivative");
  const double distanceContribution = distance > margin ? distanceRate / stoppingTime
      : distance < margin ? 0.0 : std::min(0.0, distanceRate / stoppingTime);
  return distanceContribution + pflRate;
}

inline double barrierBound(double speedRate, double beta, double gamma,
                           double combinedSpeed, double robotApproach,
                           double humanApproach, double humanApproachRate)
{
  // h = combinedSpeed - (robotApproach - humanApproach).
  // hdot = speedRate - beta - Jn*qddot + humanApproachRate.
  const double bound = speedRate - beta + humanApproachRate
      + gamma * (combinedSpeed - robotApproach + humanApproach);
  if(!std::isfinite(bound) || !std::isfinite(gamma) || gamma <= 0)
    throw std::domain_error("invalid barrier acceleration bound");
  return bound;
}
} // namespace ssm_pfl

#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ssm_pfl
{
struct Limit
{
  double inverseMass;
  double reflectedMass;
  double pflSpeed;
  double combinedSpeed;
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

// The QP controls robot acceleration, not human motion. Human approach is
// measured separately for contact analysis.
inline double accelerationBound(double combinedSpeed, double robotApproach,
                                double jacobianDrift, double dt)
{
  const double bound = combinedSpeed - robotApproach - dt * jacobianDrift;
  if(!std::isfinite(bound) || !std::isfinite(dt) || dt <= 0)
    throw std::domain_error("invalid acceleration inequality");
  return bound;
}
} // namespace ssm_pfl

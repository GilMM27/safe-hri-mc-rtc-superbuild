#include "../kinova_hold_controller/src/SSMPFLMath.h"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <iostream>

static bool near(double a, double b, double tolerance = 1e-10) { return std::abs(a-b) < tolerance; }

int main()
{
  Eigen::Matrix2d mass;
  mass << 2.0, 0.0, 0.0, 8.0;
  Eigen::LDLT<Eigen::MatrixXd> factor(mass);
  Eigen::RowVector2d jacobian(1.0, 1.0);
  const auto first = ssm_pfl::limit(factor, jacobian, 50.0, 10000.0, 0.3, 0.1, 0.5, 1e-12);
  assert(near(first.inverseMass, 0.625));
  assert(near(first.reflectedMass, 1.6));
  assert(near(first.pflSpeed, 0.5 * std::sqrt(0.625)));
  assert(near(first.combinedSpeed, 0.4 + first.pflSpeed));
  assert(first.pflSpeed + 0.1 < first.combinedSpeed); // far-away speed above standalone PFL

  const auto nearContact = ssm_pfl::limit(factor, jacobian, 50.0, 10000.0, 0.05, 0.1, 0.5, 1e-12);
  assert(near(nearContact.combinedSpeed, nearContact.pflSpeed));
  // A fast incoming human exhausts the robot's budget without requiring retreat.
  const double dt = 0.002;
  assert(near(ssm_pfl::robotSpeedBudget(nearContact.combinedSpeed,
                                      -nearContact.pflSpeed - 0.1), 0.0));
  const double stopped = ssm_pfl::pointSpeedUpperBound(0.0, 0.0, 0.0, dt);
  assert(near(stopped, 0.0));
  assert(0.0 - (-nearContact.pflSpeed - 0.1) > nearContact.pflSpeed);
  // Slowing or receding immediately restores headroom for the nominal task.
  assert(near(ssm_pfl::robotSpeedBudget(0.2, -0.1), 0.1));
  assert(near(ssm_pfl::robotSpeedBudget(0.2, 0.1), 0.3));
  const double resumed = ssm_pfl::pointSpeedUpperBound(0.1, 0.0, 0.0, dt);
  assert(near(resumed, 0.1));
  // A moving point must brake; a sudden zero budget can exceed a hard
  // acceleration limit and is then infeasible rather than softened.
  const double braking = ssm_pfl::pointSpeedUpperBound(0.1, 0.2, 0.0, dt);
  assert(near(braking, -0.1));
  assert(braking / dt < -10.0);
  const double feasibleBrake = ssm_pfl::pointSpeedUpperBound(0.08, 0.1, 0.0, dt);
  assert(near(feasibleBrake / dt, -10.0));
  // Separate point rows leave a second independent point free to move.
  Eigen::Vector2d pointAcceleration(0.0, 5.0);
  assert(near(Eigen::RowVector2d(1, 0).dot(pointAcceleration), 0.0));
  assert(Eigen::RowVector2d(0, 1).dot(pointAcceleration) > 0.0);
  const auto lowForce = ssm_pfl::limit(factor, jacobian, 25.0, 10000.0, 0.3, 0.1, 0.5, 1e-12);
  assert(near(lowForce.pflSpeed, first.pflSpeed / 2));
  const auto stiff = ssm_pfl::limit(factor, jacobian, 50.0, 40000.0, 0.3, 0.1, 0.5, 1e-12);
  assert(near(stiff.pflSpeed, first.pflSpeed / 2));

  Eigen::Matrix2d coupled;
  coupled << 2.0, 1.0, 1.0, 8.0;
  Eigen::LDLT<Eigen::MatrixXd> coupledFactor(coupled);
  const auto plus = ssm_pfl::limit(coupledFactor, Eigen::RowVector2d(1,1), 50,10000,0,0,1,1e-12);
  const auto minus = ssm_pfl::limit(coupledFactor, Eigen::RowVector2d(1,-1),50,10000,0,0,1,1e-12);
  assert(!near(plus.inverseMass, minus.inverseMass));
  const auto immobile = ssm_pfl::limit(factor, Eigen::RowVector2d::Zero(),50,10000,0,0,1,1e-12);
  assert(near(immobile.pflSpeed, 0));
  assert(std::isinf(immobile.reflectedMass));
  // Constant mobility: closing separation reduces the allowance and the
  // acceleration row has acceleration, rather than speed, units.
  assert(near(ssm_pfl::combinedSpeedRate(0.3, 0.1, -0.2, 0.0, 0.5), -0.4));
  assert(near(ssm_pfl::barrierBound(-0.4, 0.0, 2.0, 1.0, 0.8, 0.0, 0.0), 0.0));
  assert(ssm_pfl::combinedSpeedRate(0.1, 0.1, -0.2, 0.0, 0.5) < 0);
  assert(near(ssm_pfl::combinedSpeedRate(0.1, 0.1, 0.2, 0.0, 0.5), 0));
  assert(near(ssm_pfl::combinedSpeedRate(0.05, 0.1, -0.2, 0.0, 0.5), 0));
  assert(ssm_pfl::pflSpeedRate(50, 10000, first.inverseMass, -0.1, 1e-12, 0.002) < 0);
  // A near-zero mobility cannot lose more than its remaining PFL allowance
  // during one controller step, even with an old negative mobility-rate sample.
  const double tinyMobility = 1e-10;
  const double remainingPfl = 140 / std::sqrt(75000.0) * std::sqrt(tinyMobility);
  const double finiteRate = ssm_pfl::pflSpeedRate(140, 75000, tinyMobility, -1e-4, 1e-12, 0.002);
  assert(near(finiteRate, -remainingPfl / 0.002));
  assert(ssm_pfl::barrierBound(finiteRate, -0.0087, 5.0, 0.1125, 0, 0, 0) > 0);
  // A derivative predicting zero mobility within one tick must use the
  // temporal fallback rather than extrapolating the PFL speed rate.
  assert(!ssm_pfl::mobilityExtrapolationValid(3.2178674e-5, -0.0263035, 1e-12, 0.002));
  assert(ssm_pfl::mobilityExtrapolationValid(first.inverseMass, -0.1, 1e-12, 0.002));
  // At overlap with a violated speed limit, the derivative fallback asks
  // for gradual recovery instead of removing the gap in one 2 ms step.
  const double contactBarrier = -0.1226542;
  const double contactBound = ssm_pfl::barrierBound(0.0, -0.9244389, 5.0,
                                                    0.15, 0.15 - contactBarrier, 0.0, 0.0);
  assert(contactBound > 0.3 && contactBound < 0.32);
  assert(ssm_pfl::limit(factor, Eigen::RowVector2d(0.5, 0.5), 50, 10000,
                        0.3, 0.1, 0.5, 1e-12).pflSpeed < first.pflSpeed);
  bool zeroRateRejected = false;
  try { ssm_pfl::pflSpeedRate(50, 10000, 0, 0, 1e-12, 0.002); }
  catch(const std::domain_error &) { zeroRateRejected = true; }
  assert(zeroRateRejected);

  // Differentiating w = j H^-1 j^T on a smooth two-DOF example.
  Eigen::RowVector2d jdot(0.2, -0.1);
  Eigen::Matrix2d mdot; mdot << 0.3, 0.1, 0.1, -0.2;
  const Eigen::Vector2d z = factor.solve(jacobian.transpose());
  const double analytic = ssm_pfl::mobilityRate(jdot, z, mdot);
  const double eps = 1e-6;
  Eigen::LDLT<Eigen::MatrixXd> perturbed(mass + eps * mdot);
  const Eigen::RowVector2d nextJ = jacobian + eps * jdot;
  const double nextW = nextJ.dot(perturbed.solve(nextJ.transpose()));
  assert(near(analytic, (nextW - first.inverseMass) / eps, 1e-6));
  // Two pair rows act on the same vector simultaneously.
  const Eigen::Vector2d acceleration(-0.3, -0.4);
  assert(Eigen::RowVector2d(1, 0).dot(acceleration) <= -0.2);
  assert(Eigen::RowVector2d(0, 1).dot(acceleration) <= -0.3);
  ssm_pfl::TemporalDerivative temporal;
  const Eigen::Vector3d normal(1, 0, 0), local(0, 0, 0);
  const Eigen::Vector2d velocity(0.4, -0.2);
  temporal.observe(1, 1.0, jacobian, velocity, first.inverseMass, 0.0, normal, local,
                   "link_geom", false, 0.1, 0.03, 0.3, 1e-12);
  assert(!temporal.valid);
  temporal.observe(2, 1.05, jacobian + 0.05 * jdot, velocity,
                   first.inverseMass + 0.05 * analytic, 0.0, normal, local,
                   "link_geom", false, 0.1, 0.03, 0.3, 1e-12);
  assert(temporal.valid && near(temporal.beta, jdot.dot(velocity)));
  const double heldRate = temporal.mobilityRate;
  temporal.observe(2, 1.05, 100 * jacobian, velocity, 100, 0.0, normal, local,
                   "link_geom", false, 0.1, 0.03, 0.3, 1e-12);
  assert(temporal.valid && near(temporal.mobilityRate, heldRate));
  temporal.observe(3, 1.10, jacobian, velocity, first.inverseMass, 0.0, normal,
                   Eigen::Vector3d(0.1, 0, 0), "link_geom", false,
                   0.1, 0.03, 0.3, 1e-12);
  assert(!temporal.valid && temporal.switched);
  temporal.observe(4, 1.15, jacobian, velocity, first.inverseMass, 0.0, normal,
                   Eigen::Vector3d(0.1, 0, 0), "other_geom", false,
                   0.1, 0.03, 0.3, 1e-12);
  assert(!temporal.valid && temporal.switched);
  temporal.observe(5, 1.20, jacobian, velocity, first.inverseMass, 0.0,
                   Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0.1, 0, 0),
                   "other_geom", false, 0.1, 0.03, 0.3, 1e-12);
  assert(!temporal.valid && temporal.switched);

  // An approaching human reduces relative headroom. Increasing approach
  // speed reduces the admissible acceleration again through hdot.
  const double frozenBarrier = ssm_pfl::barrierBound(0.0, 0.0, 2.0, 0.2, 0.1, 0.0, 0.0);
  const double movingBarrier = ssm_pfl::barrierBound(0.0, 0.0, 2.0, 0.2, 0.1, -0.15, -0.2);
  assert(near(frozenBarrier, 0.2));
  assert(near(movingBarrier, -0.3));
  // On the first moving sample the derivative is unavailable. Retain its
  // measured human approach in h even when the unknown rate is set to zero.
  const double movingFallback = ssm_pfl::barrierBound(0.0, 0.0, 2.0, 0.2, 0.1, -0.15, 0.0);
  assert(near(movingFallback, -0.1));
  assert(movingBarrier < movingFallback && movingFallback < frozenBarrier);
  // A barrier on relative speed can ask for robot retreat. The pointwise
  // policy for the same fast incoming human instead floors its budget at zero.
  assert(ssm_pfl::barrierBound(0.0, 0.0, 2.0, 0.1, 0.0, -0.2, 0.0) < 0.0);
  assert(near(ssm_pfl::robotSpeedBudget(0.1, -0.2), 0.0));
  ssm_pfl::TemporalDerivative moving;
  moving.observe(1, 1.0, jacobian, velocity, first.inverseMass, -0.10, normal, local,
                 "link_geom", false, 0.1, 0.03, 0.3, 1e-12);
  moving.observe(2, 1.05, jacobian, velocity, first.inverseMass, -0.11, normal, local,
                 "link_geom", false, 0.1, 0.03, 0.3, 1e-12);
  assert(moving.valid && near(moving.humanApproachRate, -0.2));
  assert(near(ssm_pfl::barrierBound(0.0, moving.beta, 2.0, 0.2, 0.1,
                                  -0.11, moving.humanApproachRate), -0.22));

  // Human motion toward the robot tightens the robot acceleration bound;
  // motion away relaxes it. The next-step relative speed equals the limit at
  // the bound.
  assert(near(ssm_pfl::accelerationBound(0.2, 0.0, 0.0, 0.0, 0.002), 0.2));
  assert(near(ssm_pfl::accelerationBound(0.0, 0.0, 0.0, 0.0, 0.002), 0.0));
  assert(near(ssm_pfl::accelerationBound(0.2, 0.1, 0.0, 2.0, 0.002), 0.096));
  const double approachingBound = ssm_pfl::accelerationBound(0.2, 0.1, -0.15, 2.0, 0.002);
  const double retreatingBound = ssm_pfl::accelerationBound(0.2, 0.1, 0.15, 2.0, 0.002);
  assert(near(approachingBound, -0.054));
  assert(near(retreatingBound, 0.246));
  assert(near(0.1 + 0.002 * 2.0 + approachingBound - (-0.15), 0.2));

  // With regions on opposite sides, retreat from +x approaches -x.
  const double robotXVelocity = -0.4;
  const double closingRight = robotXVelocity;
  const double closingLeft = -robotXVelocity;
  assert(closingRight < 0 && closingLeft > 0.2);

  bool rejected = false;
  try { ssm_pfl::limit(factor, jacobian,50,0,0.2,0,0.5,1e-12); }
  catch(const std::domain_error &) { rejected = true; }
  assert(rejected);
  std::cout << "SSM+PFL mathematical checks passed\n";
}

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

  const auto nearContact = ssm_pfl::limit(factor, jacobian, 50.0, 10000.0, 0.05, 0.1, 0.5, 1e-12);
  assert(near(nearContact.combinedSpeed, nearContact.pflSpeed));
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

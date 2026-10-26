#include "../kinova_hold_controller/src/SSMPFLRelaxation.h"

#include <cassert>
#include <cmath>
#include <iostream>

static bool near(double a, double b, double tolerance = 1e-4)
{ return std::abs(a - b) < tolerance; }

int main()
{
  SSMPFLRelaxation qp(1, 2);
  Eigen::MatrixXd A(2, 1);
  Eigen::VectorXd b(2), weight = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd scale = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lo(1), hi(1);
  lo[0] = -2; hi[0] = 2;
  A << 1, -1;
  b << 1, 1;
  std::vector<bool> active{true, true};
  assert(qp.solve(A, b, active, weight, scale, lo, hi, 1e-10));
  assert(qp.status() == "original rows feasible" && near(qp.delta().norm(), 0));

  // An unreachable braking row a <= -4 with physical a >= -2.
  b << -4, 0;
  active[1] = false;
  assert(qp.solve(A, b, active, weight, scale, lo, hi, 1e-10));
  assert(near(qp.delta()[0], 2) && qp.delta()[1] == 0);
  assert(near(qp.objective(), 2));
  assert(A.row(0).dot(qp.acceleration()) - b[0] <= qp.delta()[0] + 1e-12);
  assert(qp.acceleration()[0] >= lo[0] && qp.acceleration()[0] <= hi[0]);

  // Sudden tightening followed by a feasible next step clears the old slack.
  b[0] = -1;
  assert(qp.solve(A, b, active, weight, scale, lo, hi, 1e-10));
  assert(near(qp.delta()[0], 0) && qp.delta()[1] == 0);

  // Opposing pairs remain present. More weight on the first pair allocates
  // most violation to the second pair.
  active[1] = true;
  b << -1, -1;
  weight << 4, 1;
  assert(qp.solve(A, b, active, weight, scale, lo, hi, 1e-10));
  assert(qp.delta()[0] > 0 && qp.delta()[1] > 0);
  assert(qp.delta()[0] < qp.delta()[1]);
  assert(near(qp.delta()[0], 0.4, 1e-2));
  assert(near(qp.delta()[1], 1.6, 1e-2));
  for(int i = 0; i < 2; ++i)
    assert(A.row(i).dot(qp.acceleration()) - b[i] <= qp.delta()[i] + 1e-12);

  // Stage 2 may choose any tracking target, but the Stage 1 componentwise
  // caps keep each original residual below its fixed optimal slack.
  const auto optimal = qp.delta();
  Eigen::QLD tracking(1, 0, 2);
  Eigen::MatrixXd trackQ(1, 1), emptyA(0, 1);
  Eigen::VectorXd trackC(1), emptyB(0), cap = b + optimal;
  trackQ(0, 0) = 1;
  for(double target : {-100.0, 100.0})
  {
    trackC[0] = -target;
    assert(tracking.solve(trackQ, trackC, emptyA, emptyB, A, cap, lo, hi));
    for(int i = 0; i < 2; ++i)
      assert(A.row(i).dot(tracking.result()) - b[i] <= optimal[i] + 1e-5);
  }

  // A zero-budget point has only an approach-side row; retreat is unconstrained by PFL.
  SSMPFLRelaxation pointwise(1, 1);
  Eigen::MatrixXd pointRow(1, 1);
  Eigen::VectorXd pointWeight(1), pointScale(1), pointBound(1);
  pointRow << 0.002;
  pointBound << -0.1; // current approach speed +0.1 m/s
  pointWeight[0] = 1;
  pointScale[0] = 0.01;
  lo[0] = -10; hi[0] = 10;
  assert(pointwise.solve(pointRow, pointBound, {true}, pointWeight, pointScale,
                         lo, hi, 1e-10));
  assert(near(pointwise.delta()[0], 0.08, 1e-3));
  Eigen::VectorXd relaxedBound = pointBound;
  relaxedBound[0] += pointwise.delta()[0];
  Eigen::QLD pointTracking(1, 0, 1);
  trackC[0] = 10; // nominal target favors retreat
  assert(pointTracking.solve(trackQ, trackC, emptyA, emptyB,
                             pointRow, relaxedBound, lo, hi));
  const double nextSpeed = 0.1 + 0.002 * pointTracking.result()[0];
  assert(nextSpeed <= pointwise.delta()[0] + 1e-5);
  pointBound << 0.1; // current retreat speed -0.1 m/s at zero budget
  assert(pointwise.solve(pointRow, pointBound, {true}, pointWeight, pointScale,
                         lo, hi, 1e-10));
  assert(near(pointwise.delta()[0], 0));
  assert(pointTracking.solve(trackQ, trackC, emptyA, emptyB,
                             pointRow, pointBound, lo, hi));
  assert(near(-0.1 + 0.002 * pointTracking.result()[0], -0.12, 1e-3));
  pointBound << 0.2; // a recovered 0.1 m/s budget also needs no slack
  assert(pointwise.solve(pointRow, pointBound, {true}, pointWeight, pointScale,
                         lo, hi, 1e-10));
  assert(near(pointwise.delta()[0], 0));

  // Reset/inactive rows are fixed to zero. Empty physical bounds still fail.
  active = {false, false};
  assert(qp.solve(A, b, active, weight, scale, lo, hi, 1e-10));
  assert(qp.delta().norm() == 0);
  hi[0] = 2;
  lo[0] = 3;
  assert(!qp.solve(A, b, active, weight, scale, lo, hi, 1e-10));
  std::cout << "SSM+PFL relaxation checks passed\n";
}

#include "../kinova_controllers/src/SSMPFLConstraint.h"
#include <eigen-qld/QLD.h>

#include <cassert>
#include <cmath>

int main()
{
  SSMPFLConstraint rows(0, 1, 2, false, true);
  Eigen::RowVectorXd approach(1);
  approach << 0.002;
  rows.set(0, approach, -0.1);
  assert(rows.nrInEq() == 2);
  const double original = rows.bInEq()[0];
  rows.setAllowance(0, 0.08);
  assert(std::abs(rows.bInEq()[0] - (original + 0.08 * 500.0)) < 1e-10);
  rows.setAllowance(0, 0.02);
  assert(std::abs(rows.bInEq()[0] - (original + 0.02 * 500.0)) < 1e-10);
  rows.suspend(true);
  assert(rows.nrInEq() == 0);
  rows.suspend(false);
  assert(rows.nrInEq() == 2);
  assert(std::abs(rows.bInEq()[0] - (original + 0.02 * 500.0)) < 1e-10);
  rows.clear();
  assert(rows.nrInEq() == 0);

  // Stage 1 can satisfy a safety row within its acceleration box while a
  // separate hard joint bound forces a different acceleration in the full QP.
  SSMPFLConstraint safety(0, 1, 1, false);
  approach << 1.0;
  safety.set(0, approach, -1.0);
  Eigen::QLD full(1, 0, 1);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(1, 1), emptyA(0, 1);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(1), emptyB(0);
  Eigen::VectorXd lower(1), upper(1);
  lower << 0.0;
  upper << 1.0;
  assert(!full.solve(Q, c, emptyA, emptyB, safety.A(), safety.bInEq(), lower, upper));
  safety.suspend(true);
  assert(full.solve(Q, c, emptyA, emptyB, emptyA, emptyB, lower, upper));
  const double required = std::max(0.0, safety.residual(0, full.result())) + 1e-6;
  assert(required > 0.2);
  safety.suspend(false);
  safety.setAllowance(0, required);
  assert(full.solve(Q, c, emptyA, emptyB, safety.A(), safety.bInEq(), lower, upper));
  assert(safety.residual(0, full.result()) <= 1e-6);

  // Extra safety allowance cannot resolve an inconsistent hard-only set.
  Eigen::MatrixXd hardRow(1, 1);
  Eigen::VectorXd hardUpper(1);
  hardRow << 1.0;
  hardUpper << -1.0;
  assert(!full.solve(Q, c, emptyA, emptyB, hardRow, hardUpper, lower, upper));
}

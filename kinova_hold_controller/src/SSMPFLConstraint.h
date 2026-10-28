#pragma once

#include <mc_solver/GenInequalityConstraint.h>
#include <mc_solver/InequalityConstraint.h>
#include <algorithm>

// Tasks backend only. One row per human region and robot link, with fixed storage
// so a changing closest pair does not resize the QP on each controller tick.
struct SSMPFLConstraint final : mc_solver::InequalityConstraintRobot
{
  SSMPFLConstraint(unsigned int robotIndex, int dofs, int pairs, bool accelerationUnits,
                   bool normalizeRows = false)
  : mc_solver::InequalityConstraintRobot(robotIndex), accelerationUnits_(accelerationUnits),
    normalizeRows_(normalizeRows),
    rawA_(Eigen::MatrixXd::Zero(pairs, dofs)),
    rawUpper_(Eigen::VectorXd::Zero(pairs)),
    baseUpper_(Eigen::VectorXd::Zero(pairs)),
    rowScale_(Eigen::VectorXd::Ones(pairs)),
    A_(Eigen::MatrixXd::Zero(pairs, dofs)),
    upper_(Eigen::VectorXd::Zero(pairs)) {}

  void clear()
  {
    active_ = false;
    suspended_ = false;
    A_.setZero();
    rawA_.setZero();
    rawUpper_.setZero();
    baseUpper_.setZero();
    rowScale_.setOnes();
    upper_.setZero();
  }
  void set(int row, const Eigen::RowVectorXd & accelerationRow, double bound)
  {
    active_ = true;
    rawA_.row(row) = accelerationRow;
    rawUpper_[row] = bound;
    baseUpper_[row] = bound;
    rowScale_[row] = normalizeRows_ ? 1.0 / std::max(1e-3, accelerationRow.norm()) : 1.0;
    A_.row(row) = rowScale_[row] * accelerationRow;
    upper_[row] = rowScale_[row] * bound;
  }
  double residual(int row, const Eigen::VectorXd & acceleration) const
  { return rawA_.row(row).dot(acceleration) - rawUpper_[row]; }
  double upperResidual(int row, const Eigen::VectorXd & acceleration) const
  { return rawA_.row(row).dot(acceleration) - rawUpper_[row]; }
  const Eigen::MatrixXd & rows() const { return rawA_; }
  const Eigen::VectorXd & bounds() const { return rawUpper_; }
  void relax(int row, double amount)
  { rawUpper_[row] += amount; upper_[row] += rowScale_[row] * amount; }
  void setAllowance(int row, double amount)
  { rawUpper_[row] = baseUpper_[row] + amount; upper_[row] = rowScale_[row] * rawUpper_[row]; }
  void suspend(bool value) { suspended_ = value; }
  const Eigen::MatrixXd & A() const override { return A_; }
  void compute() override {}
  int maxInEq() const override { return static_cast<int>(A_.rows()); }
  int nrInEq() const override { return active_ && !suspended_ ? maxInEq() : 0; }
  std::string nameInEq() const override { return accelerationUnits_ ? "Kinova SSM+PFL temporal acceleration barrier"
      : "Kinova SSM+PFL directional speed upper bounds"; }
  const Eigen::VectorXd & bInEq() const override { return upper_; }

private:
  // Disabled slots are exactly 0 <= 0 while other pairs are active.
  bool active_ = false;
  bool suspended_ = false;
  bool accelerationUnits_ = false;
  bool normalizeRows_ = false;
  Eigen::MatrixXd rawA_;
  Eigen::VectorXd rawUpper_, baseUpper_, rowScale_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd upper_;
};

// Opt-in model joint bounds used identically in the relaxation QP and the
// tracking QP. These are independent hard rows, never softened by delta.
struct SSMPFLAccelerationBounds final : mc_solver::GenInequalityConstraintRobot
{
  SSMPFLAccelerationBounds(unsigned int robotIndex, int dofs)
  : mc_solver::GenInequalityConstraintRobot(robotIndex),
    A_(Eigen::MatrixXd::Identity(dofs, dofs)),
    lower_(Eigen::VectorXd::Constant(dofs, -1e12)),
    upper_(Eigen::VectorXd::Constant(dofs, 1e12)) {}
  void set(const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
  { lower_ = lower; upper_ = upper; }
  void compute() override {}
  int maxGenInEq() const override { return static_cast<int>(A_.rows()); }
  std::string nameGenInEq() const override { return "Kinova recovery model joint bounds"; }
  const Eigen::MatrixXd & A() const override { return A_; }
  const Eigen::VectorXd & LowerGenInEq() const override { return lower_; }
  const Eigen::VectorXd & UpperGenInEq() const override { return upper_; }
private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd lower_, upper_;
};

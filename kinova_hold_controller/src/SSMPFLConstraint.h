#pragma once

#include <mc_solver/GenInequalityConstraint.h>

// Tasks backend only. One row per human region and robot link, with fixed storage
// so a changing closest pair does not resize the QP on each controller tick.
struct SSMPFLConstraint final : mc_solver::GenInequalityConstraintRobot
{
  SSMPFLConstraint(unsigned int robotIndex, int dofs, int pairs)
  : mc_solver::GenInequalityConstraintRobot(robotIndex),
    A_(Eigen::MatrixXd::Zero(pairs, dofs)),
    lower_(Eigen::VectorXd::Constant(pairs, -inactiveBound())),
    upper_(Eigen::VectorXd::Constant(pairs, inactiveBound())) {}

  void clear()
  {
    active_ = false;
    A_.setZero();
    upper_.setConstant(inactiveBound());
  }
  void set(int row, const Eigen::RowVectorXd & accelerationRow, double bound)
  {
    active_ = true;
    A_.row(row) = accelerationRow;
    upper_[row] = bound;
  }
  double residual(int row, const Eigen::VectorXd & acceleration) const
  {
    return A_.row(row).dot(acceleration) - upper_[row];
  }
  const Eigen::MatrixXd & A() const override { return A_; }
  void compute() override {}
  int maxGenInEq() const override { return static_cast<int>(A_.rows()); }
  int nrGenInEq() const override { return active_ ? maxGenInEq() : 0; }
  std::string nameGenInEq() const override { return "Kinova SSM+PFL directional speed"; }
  const Eigen::VectorXd & LowerGenInEq() const override { return lower_; }
  const Eigen::VectorXd & UpperGenInEq() const override { return upper_; }

private:
  // QLD requires finite bounds when the distance worker activates these rows.
  static double inactiveBound() { return 1e12; }
  bool active_ = false;
  Eigen::MatrixXd A_;
  Eigen::VectorXd lower_, upper_;
};

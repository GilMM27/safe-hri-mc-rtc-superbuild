#pragma once

#include <mc_solver/TVMQPSolver.h>
#include <tvm/function/BasicLinearFunction.h>
#include <tvm/Space.h>
#include <tvm/task_dynamics/None.h>
#include <algorithm>
#include <vector>

// Fixed storage keeps the TVM problem shape stable as closest pairs change.
// In soft mode, each active row uses a nonnegative slack in row units with a
// weighted priority-1 objective; inactive rows remain the tautology 0 <= 0.
struct SSMPFLConstraint final
{
  SSMPFLConstraint(tvm::VariablePtr acceleration, int dofs, int pairs, bool normalizeRows = false,
                   double softWeight = 0)
  : normalizeRows_(normalizeRows), softWeight_(softWeight), acceleration_(acceleration),
    rawA_(Eigen::MatrixXd::Zero(pairs, dofs)), rawUpper_(Eigen::VectorXd::Zero(pairs)),
    baseUpper_(Eigen::VectorXd::Zero(pairs)), rowScale_(Eigen::VectorXd::Ones(pairs)),
    A_(Eigen::MatrixXd::Zero(pairs, dofs)), upper_(Eigen::VectorXd::Zero(pairs)),
    slack_(softWeight > 0 ? tvm::Space(pairs).createVariable("SSMPFL row slack") : nullptr),
    slackA_(Eigen::MatrixXd::Zero(pairs, pairs)),
    function_(std::make_shared<tvm::function::BasicLinearFunction>(
        pairs, slack_ ? std::vector<tvm::VariablePtr>{acceleration, slack_}
                      : std::vector<tvm::VariablePtr>{acceleration}))
  { sync(); }

  void addToSolver(mc_solver::TVMQPSolver & solver)
  {
    task_ = solver.problem().add(function_ <= 0., tvm::task_dynamics::None(),
                                 {tvm::requirements::PriorityLevel(0)});
    if(slack_)
    {
      slackBound_ = solver.problem().add(slack_ >= 0., {tvm::requirements::PriorityLevel(0)});
      slackObjective_ = solver.problem().add(slack_ == 0.,
          {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(softWeight_)});
    }
  }
  void removeFromSolver(mc_solver::TVMQPSolver & solver)
  {
    if(slackObjective_) { solver.problem().remove(*slackObjective_); slackObjective_.reset(); }
    if(slackBound_) { solver.problem().remove(*slackBound_); slackBound_.reset(); }
    if(task_) { solver.problem().remove(*task_); task_.reset(); }
  }
  void clear()
  {
    active_ = false;
    suspended_ = false;
    A_.setZero(); rawA_.setZero(); rawUpper_.setZero(); baseUpper_.setZero();
    rowScale_.setOnes(); upper_.setZero(); slackA_.setZero();
    sync();
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
    if(slack_) slackA_(row, row) = -rowScale_[row];
    sync();
  }
  double residual(int row, const Eigen::VectorXd & acceleration) const
  { return rawA_.row(row).dot(acceleration) - rawUpper_[row]; }
  double upperResidual(int row, const Eigen::VectorXd & acceleration) const
  { return residual(row, acceleration); }
  const Eigen::MatrixXd & rows() const { return rawA_; }
  const Eigen::VectorXd & bounds() const { return rawUpper_; }
  void relax(int row, double amount)
  { rawUpper_[row] += amount; upper_[row] += rowScale_[row] * amount; sync(); }
  void setAllowance(int row, double amount)
  { rawUpper_[row] = baseUpper_[row] + amount; upper_[row] = rowScale_[row] * rawUpper_[row]; sync(); }
  void suspend(bool value) { suspended_ = value; sync(); }
  const Eigen::MatrixXd & A() const { return A_; }
  const Eigen::VectorXd & bInEq() const { return upper_; }
  double softSlack(int row) const { return slack_ ? std::max(0.0, slack_->value()[row]) : 0.0; }

private:
  void sync()
  {
    if(active_ && !suspended_)
    {
      function_->A(A_, *acceleration_);
      if(slack_) function_->A(slackA_, *slack_);
      function_->b(-upper_);
    }
    else
    {
      function_->A(Eigen::MatrixXd::Zero(A_.rows(), A_.cols()), *acceleration_);
      if(slack_) function_->A(Eigen::MatrixXd::Zero(slackA_.rows(), slackA_.cols()), *slack_);
      function_->b(Eigen::VectorXd::Zero(upper_.size()));
    }
  }
  bool active_ = false, suspended_ = false, normalizeRows_ = false;
  double softWeight_ = 0;
  tvm::VariablePtr acceleration_;
  Eigen::MatrixXd rawA_, A_;
  Eigen::VectorXd rawUpper_, baseUpper_, rowScale_, upper_;
  tvm::VariablePtr slack_;
  Eigen::MatrixXd slackA_;
  std::shared_ptr<tvm::function::BasicLinearFunction> function_;
  tvm::TaskWithRequirementsPtr task_;
  tvm::TaskWithRequirementsPtr slackBound_, slackObjective_;
};

// These hard acceleration rows mirror the Stage 1 relaxation box.
struct SSMPFLAccelerationBounds final
{
  SSMPFLAccelerationBounds(tvm::VariablePtr acceleration, int dofs)
  : A_(Eigen::MatrixXd::Zero(2 * dofs, dofs)), b_(Eigen::VectorXd::Constant(2 * dofs, -1e12)),
    function_(std::make_shared<tvm::function::BasicLinearFunction>(2 * dofs, acceleration))
  {
    A_.topRows(dofs).setIdentity();
    A_.bottomRows(dofs) = -Eigen::MatrixXd::Identity(dofs, dofs);
    function_->A(A_);
    function_->b(b_);
  }
  void addToSolver(mc_solver::TVMQPSolver & solver)
  {
    task_ = solver.problem().add(function_ <= 0., tvm::task_dynamics::None(),
                                 {tvm::requirements::PriorityLevel(0)});
  }
  void removeFromSolver(mc_solver::TVMQPSolver & solver)
  {
    if(task_) { solver.problem().remove(*task_); task_.reset(); }
  }
  void set(const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
  {
    b_.head(upper.size()) = -upper;
    b_.tail(lower.size()) = lower;
    function_->b(b_);
  }
private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  std::shared_ptr<tvm::function::BasicLinearFunction> function_;
  tvm::TaskWithRequirementsPtr task_;
};

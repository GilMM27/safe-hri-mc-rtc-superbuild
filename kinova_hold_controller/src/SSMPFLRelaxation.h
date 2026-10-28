#pragma once

#include <eigen-qld/QLD.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

// Separate QP because Tasks fixes its variable vector to robot accelerations and
// contact forces. The final mc_rtc solve still applies the ordinary task.
struct SSMPFLRelaxation
{
  SSMPFLRelaxation(int dofs, int pairs)
  : dofs_(dofs), pairs_(pairs),
    solver_(dofs + pairs, 0, pairs),
    hardSolver_(dofs, 0, pairs), hardQ_(Eigen::MatrixXd::Identity(dofs, dofs)),
    hardC_(Eigen::VectorXd::Zero(dofs)), hardA_(Eigen::MatrixXd::Zero(pairs, dofs)),
    hardEmptyA_(0, dofs),
    Q_(Eigen::MatrixXd::Zero(dofs + pairs, dofs + pairs)),
    c_(Eigen::VectorXd::Zero(dofs + pairs)),
    A_(Eigen::MatrixXd::Zero(pairs, dofs + pairs)),
    b_(Eigen::VectorXd::Zero(pairs)),
    emptyA_(0, dofs + pairs), emptyB_(0),
    lower_(Eigen::VectorXd::Zero(dofs + pairs)),
    upper_(Eigen::VectorXd::Zero(dofs + pairs)),
    delta_(Eigen::VectorXd::Zero(pairs)),
    acceleration_(Eigen::VectorXd::Zero(dofs)) {}

  bool solve(const Eigen::MatrixXd & rows, const Eigen::VectorXd & bounds,
             const std::vector<bool> & active, const Eigen::VectorXd & weights,
             const Eigen::VectorXd & scales, const Eigen::VectorXd & accelerationLower,
             const Eigen::VectorXd & accelerationUpper, double accelerationRegularization)
  {
    status_ = "invalid inputs";
    if(rows.rows() != pairs_ || rows.cols() != dofs_ || bounds.size() != pairs_
       || static_cast<int>(active.size()) != pairs_ || weights.size() != pairs_
       || scales.size() != pairs_ || accelerationLower.size() != dofs_
       || accelerationUpper.size() != dofs_ || !rows.allFinite() || !bounds.allFinite()
       || !weights.allFinite() || !scales.allFinite() || !accelerationLower.allFinite()
       || !accelerationUpper.allFinite() || !std::isfinite(accelerationRegularization)
       || accelerationRegularization <= 0) return false;
    Q_.setZero(); A_.setZero(); b_.setZero(); c_.setZero();
    for(int j = 0; j < dofs_; ++j)
    {
      if(accelerationLower[j] > accelerationUpper[j]) { status_ = "infeasible acceleration/joint bounds"; return false; }
      Q_(j, j) = accelerationRegularization;
      lower_[j] = accelerationLower[j]; upper_[j] = accelerationUpper[j];
    }
    for(int i = 0; i < pairs_; ++i)
    {
      if(weights[i] <= 0 || scales[i] <= 0) return false;
      Q_(dofs_ + i, dofs_ + i) = weights[i];
      lower_[dofs_ + i] = 0;
      upper_[dofs_ + i] = active[i] ? std::numeric_limits<double>::infinity() : 0;
      if(active[i])
      {
        A_.block(i, 0, 1, dofs_) = rows.row(i);
        A_(i, dofs_ + i) = -scales[i];
        b_[i] = bounds[i];
      }
    }
    hardA_ = rows;
    for(int i = 0; i < pairs_; ++i)
      if(!active[i])
        hardA_.row(i).setZero();
    if(hardSolver_.solve(hardQ_, hardC_, hardEmptyA_, emptyB_, hardA_, b_,
                         accelerationLower, accelerationUpper))
    {
      const auto & hard = hardSolver_.result();
      bool verified = hard.allFinite();
      for(int i = 0; i < pairs_ && verified; ++i)
        if(active[i] && rows.row(i).dot(hard) - bounds[i] > 1e-7)
          verified = false;
      if(verified)
      {
        acceleration_ = hard;
        delta_.setZero(); objective_ = 0;
        status_ = "original rows feasible";
        return true;
      }
    }
    // Inactive inequalities are 0 <= 0; their slack variable is fixed above.
    if(!solver_.solve(Q_, c_, emptyA_, emptyB_, A_, b_, lower_, upper_))
    {
      status_ = "QLD failure " + std::to_string(solver_.fail());
      return false;
    }
    const auto & x = solver_.result();
    if(!x.allFinite()) { status_ = "nonfinite QLD result"; return false; }
    // QLD may return a point a few ulps outside a bound. Clip to the hard
    // acceleration box, then make every reported slack cover this actual
    // point before passing the rows to the second, independently built QP.
    acceleration_ = x.head(dofs_).cwiseMax(accelerationLower).cwiseMin(accelerationUpper);
    for(int i = 0; i < pairs_; ++i)
    {
      const double solverSlack = scales[i] * std::max(0.0, x[dofs_ + i]);
      const double neededSlack = active[i] ? std::max(0.0, rows.row(i).dot(acceleration_) - bounds[i]) : 0;
      if(active[i] && neededSlack - solverSlack > 1e-5)
      { status_ = "QLD row residual exceeds tolerance"; return false; }
      delta_[i] = active[i] ? std::max(solverSlack, neededSlack) : 0;
    }
    objective_ = 0;
    for(int i = 0; i < pairs_; ++i)
      objective_ += 0.5 * weights[i] * std::pow(delta_[i] / scales[i], 2);
    status_ = "success";
    return true;
  }

  const Eigen::VectorXd & delta() const { return delta_; }
  const Eigen::VectorXd & acceleration() const { return acceleration_; }
  double objective() const { return objective_; }
  const std::string & status() const { return status_; }

private:
  int dofs_, pairs_;
  Eigen::QLD solver_;
  Eigen::QLD hardSolver_;
  Eigen::MatrixXd hardQ_, hardA_, hardEmptyA_;
  Eigen::VectorXd hardC_;
  Eigen::MatrixXd Q_, A_, emptyA_;
  Eigen::VectorXd c_, b_, emptyB_, lower_, upper_, delta_, acceleration_;
  double objective_ = 0;
  std::string status_ = "not run";
};

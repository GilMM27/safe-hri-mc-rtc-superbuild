#pragma once

#include <tvm/LinearizedControlProblem.h>
#include <tvm/Variable.h>
#include <tvm/scheme/WeightedLeastSquares.h>
#include <Eigen/Core>
#include <vector>

namespace ssm_pfl
{
// Solve the current full TVM problem without advancing the mc_rtc robot state.
// TVM writes optimization variables on success and failure, so restore them
// before the controller retries or performs its final integrating solve.
inline bool probeTVM(tvm::LinearizedControlProblem & problem,
                     tvm::scheme::WeightedLeastSquares & solver,
                     const tvm::VariablePtr & acceleration,
                     Eigen::VectorXd & candidate)
{
  const auto variables = problem.variables();
  std::vector<Eigen::VectorXd> saved;
  saved.reserve(variables.numberOfVariables());
  for(const auto & variable : variables) saved.push_back(variable->value());
  auto restore = [&]() {
    for(int i = 0; i < variables.numberOfVariables(); ++i) variables[i]->set(saved[i]);
  };
  try
  {
    const bool success = solver.solve(problem);
    if(success) candidate = acceleration->value();
    restore();
    return success;
  }
  catch(...)
  {
    restore();
    throw;
  }
}
} // namespace ssm_pfl

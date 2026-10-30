#include "../src/SSMPFLConstraint.h"
#include "../src/SSMPFLTVMProbe.h"
#include <tvm/Space.h>
#include <tvm/solver/defaultLeastSquareSolver.h>
#include <cmath>
#include <stdexcept>

int main()
{
  tvm::VariablePtr acceleration = tvm::Space(2).createVariable("test acceleration");
  mc_solver::TVMQPSolver solver(0.001);
  auto & problem = solver.problem();
  const Eigen::Vector2d target(1.0, 0.0);
  problem.add(acceleration == target, tvm::task_dynamics::None(),
              {tvm::requirements::PriorityLevel(1)});
  SSMPFLConstraint safety(acceleration, 2, 2, true);
  SSMPFLAccelerationBounds bounds(acceleration, 2);
  safety.addToSolver(solver);
  bounds.addToSolver(solver);
  tvm::scheme::WeightedLeastSquares optimizer(tvm::solver::DefaultLSSolverOptions{});
  auto check = [&](double expected) {
    if(!optimizer.solve(problem)) throw std::runtime_error("TVM solve failed");
    if(std::abs(acceleration->value()[0] - expected) > 1e-5)
      throw std::runtime_error("unexpected TVM acceleration");
  };

  safety.set(0, Eigen::RowVector2d(2.0, 0.0), 0.4);
  check(0.2);
  safety.relax(0, 0.6);
  check(0.5);
  safety.suspend(true);
  check(1.0);
  safety.suspend(false);
  check(0.5);
  bounds.set(Eigen::Vector2d::Constant(-0.1), Eigen::Vector2d::Constant(0.1));
  check(0.1);
  safety.clear();
  check(0.1);
  bounds.set(Eigen::Vector2d::Constant(-1e12), Eigen::Vector2d::Constant(1e12));
  check(1.0);
  const Eigen::Vector2d before(-0.3, 0.05);
  acceleration->set(before);
  Eigen::VectorXd candidate;
  if(!ssm_pfl::probeTVM(problem, optimizer, acceleration, candidate)
     || std::abs(candidate[0] - 1.0) > 1e-5
     || (acceleration->value() - before).norm() > 1e-12)
    throw std::runtime_error("feasible TVM probe changed the controller variables");
  safety.set(0, Eigen::RowVector2d(1.0, 0.0), -2.0);
  bounds.set(Eigen::Vector2d::Constant(-0.1), Eigen::Vector2d::Constant(0.1));
  if(ssm_pfl::probeTVM(problem, optimizer, acceleration, candidate)
     || (acceleration->value() - before).norm() > 1e-12)
    throw std::runtime_error("failed TVM probe changed the controller variables");
  safety.removeFromSolver(solver);
  bounds.removeFromSolver(solver);
  SSMPFLConstraint softLow(acceleration, 2, 2, true, 1.0);
  softLow.addToSolver(solver);
  softLow.set(0, Eigen::RowVector2d(2.0, 0.0), 0.0);
  check(0.2);
  if(std::abs(softLow.softSlack(0) - 0.4) > 1e-5)
    throw std::runtime_error("soft SSM+PFL slack does not match violation");
  softLow.removeFromSolver(solver);
  SSMPFLConstraint softHigh(acceleration, 2, 2, true, 9.0);
  softHigh.addToSolver(solver);
  softHigh.set(0, Eigen::RowVector2d(2.0, 0.0), 0.0);
  check(1.0 / 37.0);
  softHigh.set(0, Eigen::RowVector2d(2.0, 0.0), 2.0);
  check(1.0);
  if(std::abs(softHigh.softSlack(0)) > 1e-5)
    throw std::runtime_error("soft SSM+PFL penalized a satisfied row");
  softHigh.removeFromSolver(solver);
}

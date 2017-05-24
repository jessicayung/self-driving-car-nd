/*
 * Code heavily adapted from Udacity's code, e.g.
 * CarND-MPC-Quizzes: mpc_to_line
 */
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <math.h>

using CppAD::AD;

// TODO: Set the timestep length and duration
// size_t: type returned by sizeof, widely used to represent sizes and counts
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Reference cross-track error and orientation error = 0
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 40;

// Mark when each variable starts for convenience
// since state and actuator variables are stored in one vector
// in the format 'x...(N)x y...(N)y...'
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of cost and constraints,
    // vars is a vector containing state and actuator var values and constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
    //
    // Reference state cost
    //
    
    // Initialise cost to zero
    fg[0] = 0;
    
    // (1) Cost increases with distance from reference state
    for (int i=0; i < N; i++) {
      fg[0] += pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += pow(vars[v_start + i] - ref_v, 2);

    }
    
    // (2) Cost increases with use of actuators
    for (int i=0; i < N-1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2);
    }
    
    // (3) Cost increases with value gap between sequential actuators
    for (int i=0; i < N-2; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    //
    // Constraints
    //
    
    
    /**
    x = x + v*cos(psi)*dt;
    y = y + v*sin(psi)*dt;
    psi = psi + (v/Lf)*delta*dt;
    v = v + a*dt;
    cte_now = reference_trajectory - y;
    cte = cte_now + v + sin(epsi)*dt;
    epsi_now = psi-psi_desired;
    */
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // State: [x,y,ψ,v,cte,eψ]
  // Actuators: [δ,a]
  size_t n_vars = 6*N + 2*(N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = 0;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (int i = 0; i < 6*N; i++) {
    vars_upperbound = 1.0e18;
    vars_lowerbound = -1.0e18;
  }

  // Steering angle (deltas)
  for (int i = 6*N; i < n_vars-(N-1); ++i)
  {
    vars_upperbound = M_PI/2;
    vars_lowerbound = -M_PI/2;
  }

  // Acceleration
  for (int i = n_vars-(N-1); i < n_vars; ++i)
  {
    vars_upperbound = 1.0;
    vars_lowerbound = -1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}

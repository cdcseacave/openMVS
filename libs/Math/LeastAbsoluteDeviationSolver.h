////////////////////////////////////////////////////////////////////
// LeastAbsoluteDeviationSolver.h
//
// Solver for least absolute deviation (L1) problems using ADMM
// Based on the COLMAP implementation: https://github.com/colmap/colmap/raw/refs/heads/main/src/colmap/optim/least_absolute_deviations.h
// Copyright COLMAP 2025 - BSD license

#ifndef _MATH_LEAST_ABSOLUTE_DEVIATION_SOLVER_H_
#define _MATH_LEAST_ABSOLUTE_DEVIATION_SOLVER_H_


// I N C L U D E S /////////////////////////////////////////////////

#include <Eigen/Sparse>
#include <memory>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

struct LeastAbsoluteDeviationLinearSolverImpl;

// Least absolute deviations (LAD) fitting via ADMM by solving the problem:
//
//        min || A x - b ||_1
//
// The solution is returned in the vector x and the iterative solver is
// initialized with the given value. This implementation is based on the paper
// "Distributed Optimization and Statistical Learning via the Alternating
// Direction Method of Multipliers" by Boyd et al. and the Matlab implementation
// at https://web.stanford.edu/~boyd/papers/admm/least_abs_deviations/lad.html
struct MATH_API LeastAbsoluteDeviationSolver
{
	struct Options
	{
		// Augmented Lagrangian parameter.
		double rho = 1.0;

		// Over-relaxation parameter, typical values are between 1.0 and 1.8.
		double alpha = 1.0;

		// Maximum solver iterations.
		int max_num_iterations = 1000;

		// Absolute and relative solution thresholds, as suggested by Boyd et al.
		double absolute_tolerance = 1e-4;
		double relative_tolerance = 1e-2;

		enum class SolverType {
			SimplicialLLT,
			SupernodalCholmodLLT,
		};
		SolverType solver_type = SolverType::SimplicialLLT;
	};

	LeastAbsoluteDeviationSolver(const Options& options,
	                             const Eigen::SparseMatrix<double>& A);

	bool Solve(const Eigen::VectorXd& b, Eigen::VectorXd* x) const;

	private:
	const Options& options_;
	const Eigen::SparseMatrix<double>& A_;
	const std::shared_ptr<LeastAbsoluteDeviationLinearSolverImpl> linear_solver_;
};
/*----------------------------------------------------------------*/

// Test the solver with a simple least absolute deviation problem
MATH_API bool TestLeastAbsoluteDeviationSolver();
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // _MATH_LEAST_ABSOLUTE_DEVIATION_SOLVER_H_

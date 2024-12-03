#pragma once

#include <vector>
#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"

#include "state.hpp"
#include "polynomial.hpp"

class MinSnapTraj
{
public:
	struct Waypoint
	{
		Waypoint(): pos(Eigen::Vector3d::Zero()), yaw(0.0)
		{};
		Waypoint(Eigen::Vector3d pos, double yaw): pos(pos), yaw(yaw)
		{

		};
		Eigen::Vector3d pos;
		double yaw;
	};
	MinSnapTraj();
	void AddWaypoint(Waypoint new_waypoint);
	void ClearWaypoints();
	bool Solve(double average_speed);
	void Evaluate(double time, State &state);
	double EndTime();
	bool GetWaypoint(int waypoint_num, Waypoint &to_fill);
	int GetWaypointCount() { return int(waypoints_.size()); }
	bool solved() { return solved_; }

	
private:
	void CalculateTimes(double average_speed);
	void CalculateTimePowers(Eigen::MatrixXd &time_powers);
	void GenerateQ(const Eigen::MatrixXd &time_powers, Eigen::SparseMatrix<double> &Q);
	void FillH(const Eigen::VectorXd &time_powers_row, Eigen::MatrixXd &H);
	void GenerateConstraints(const Eigen::MatrixXd &time_powers, Eigen::SparseMatrix<double> &A, Eigen::VectorXd &b_x, Eigen::VectorXd &b_y, Eigen::VectorXd &b_z, Eigen::VectorXd &b_yaw);
	int CalculatePolyCoeffMultiplier(int coeff, int derivative_count);
	int CalculatePolyCoeffPower(int coeff, int derivative_count);
	void CalculatePolyDerivativeMultipliers(const int coeff_count, const int derivative_count, const Eigen::VectorXd &time_powers_row, Eigen::RowVectorXd &polynomial_derivative);

	bool solved_;
	double start_time_;
	double end_time_;

	const int kPosMinDerivative = 4;
	const int kPolyOrder = 7;
	const int kCoeffCount = kPolyOrder + 1;
	// n is poly order
	// m is number of "waypoints"
	// consts per solve
	int num_segments_;
	int num_internal_joints_;
	int num_variables_;
	int num_constraints_;
	Eigen::VectorXd times_;

	OsqpEigen::Solver solver_;
	bool solver_init_;

	std::vector<Waypoint> waypoints_;
	std::vector<std::vector<Polynomial>> x_polys_;
	std::vector<std::vector<Polynomial>> y_polys_;
	std::vector<std::vector<Polynomial>> z_polys_;
	std::vector<std::vector<Polynomial>> yaw_polys_;
};
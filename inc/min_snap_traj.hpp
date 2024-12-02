#pragma once
#include "state.hpp"
#include <vector>
#include <Eigen/Eigen>

class MinSnapTraj
{
public:
	struct Waypoint
	{
		Waypoint(Eigen::Vector3d pos, double yaw): pos(pos), yaw(yaw)
		{

		};
		Eigen::Vector3d pos;
		double yaw;
	};
	MinSnapTraj();
	void AddWaypoint(Waypoint new_waypoint);
	bool Solve(double average_speed);
	void Evaluate(double time);
	
private:
	void CalculateTimes(double average_speed);
	void CalculateTimePowers(Eigen::MatrixXd &time_powers);
	void GenerateQ(const Eigen::MatrixXd &time_powers, Eigen::SparseMatrix<double> &Q);
	void FillH(const Eigen::VectorXd &time_powers_row, Eigen::MatrixXd &H);
	void GenerateConstraints(const Eigen::MatrixXd &time_powers, Eigen::SparseMatrix<double> &A, Eigen::VectorXd &b_x, Eigen::VectorXd &b_y, Eigen::VectorXd &b_z);
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
	Eigen::VectorXd times_;
	Eigen::VectorXd c_;

	std::vector<Waypoint> waypoints_;
};
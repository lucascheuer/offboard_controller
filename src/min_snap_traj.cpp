#include "min_snap_traj.hpp"
#include <iostream>

MinSnapTraj::MinSnapTraj():solved_(false), start_time_(0.0), end_time_(0.0)
{

}

void MinSnapTraj::AddWaypoint(Waypoint new_waypoint)
{
	waypoints_.push_back(new_waypoint);
}
void MinSnapTraj::Evaluate(double time)
{

}
bool MinSnapTraj::Solve(double average_speed)
{
	std::cout << "solving" << std::endl;
	num_segments_ = waypoints_.size() - 1;
	num_internal_joints_ = waypoints_.size() - 2;
	
	times_.resize(num_segments_);
	
	std::cout << "calc time" << std::endl;
	CalculateTimes(average_speed);
	Eigen::MatrixXd time_powers;
	std::cout << "calc powers" << std::endl;
	CalculateTimePowers(time_powers);
	Eigen::SparseMatrix<double> Q(kCoeffCount * num_segments_, kCoeffCount * num_segments_);

	Eigen::SparseMatrix<double> A(2 * num_segments_ + 8 + 4 * num_internal_joints_, kCoeffCount * num_segments_);
	Eigen::VectorXd b_x(2 * num_segments_ + 8 + 4 * num_internal_joints_);
	Eigen::VectorXd b_y(2 * num_segments_ + 8 + 4 * num_internal_joints_);
	Eigen::VectorXd b_z(2 * num_segments_ + 8 + 4 * num_internal_joints_);
	std::cout << "generate Q" << std::endl;
	GenerateQ(time_powers, Q);
	std::cout << "Get constraints" << std::endl;
	GenerateConstraints(time_powers, A, b_x, b_y, b_z);
	Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	std::cout << "Q" << std::endl;
	std::cout << Q.toDense() << std::endl << std::endl;
	std::cout << "A" << std::endl;
	std::cout << A.toDense() << std::endl << std::endl;
	std::cout << "b x" << std::endl;
	std::cout << b_x << std::endl << std::endl;
	std::cout << "b y" << std::endl;
	std::cout << b_y << std::endl << std::endl;
	std::cout << "b z" << std::endl;
	std::cout << b_z << std::endl << std::endl;

	return true;
}
void MinSnapTraj::CalculateTimes(double average_speed)
{
	for (int waypoint_index = 1; waypoint_index < int(waypoints_.size()); ++waypoint_index)
	{
		double distance = (waypoints_.at(waypoint_index).pos - waypoints_.at(waypoint_index - 1).pos).norm();
		double time = distance / average_speed;
		times_(waypoint_index - 1) = time;
	}
}


void MinSnapTraj::CalculateTimePowers(Eigen::MatrixXd &time_powers)
{
	time_powers = Eigen::MatrixXd::Ones(num_segments_, kCoeffCount);

	for (int segment = 0; segment < num_segments_; ++segment)
	{
		time_powers(segment, 1) = times_(segment);
		for (int power = 2; power < kCoeffCount; ++power)
		{
			time_powers(segment, power) = time_powers(segment, power - 1) * times_(segment);
		}
	}
}

void MinSnapTraj::GenerateQ(const Eigen::MatrixXd &time_powers, Eigen::SparseMatrix<double> &Q)
{
	
	assert(Q.rows() == num_segments_ * kCoeffCount && Q.cols() == num_segments_ * kCoeffCount);
	Eigen::MatrixXd DenseQ = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
	for (int segment = 0; segment < num_segments_; ++segment)
	{
		Eigen::MatrixXd H = Eigen::MatrixXd::Zero(kCoeffCount, kCoeffCount);
		FillH(time_powers.row(segment), H);
		int block_location = segment * kCoeffCount;
		DenseQ.block(block_location, block_location, kCoeffCount, kCoeffCount) = H;
	}
	Q = DenseQ.sparseView();
}

void MinSnapTraj::FillH(const Eigen::VectorXd &time_powers_row, Eigen::MatrixXd &H)
{
	Eigen::Matrix4d non_zero_h;
	non_zero_h <<	1152  * time_powers_row(1), 2880  * time_powers_row(2), 5760   * time_powers_row(3), 10080  * time_powers_row(4),
					2880  * time_powers_row(2), 9600  * time_powers_row(3), 21600  * time_powers_row(4), 40320  * time_powers_row(5),
					5760  * time_powers_row(3), 21600 * time_powers_row(4), 51840  * time_powers_row(5), 100800 * time_powers_row(6),
					10080 * time_powers_row(4), 40320 * time_powers_row(5), 100800 * time_powers_row(6), 201600 * time_powers_row(7);

	H.bottomRightCorner(4, 4) = non_zero_h;
	// ⎡0  0  0  0      0          0          0           0     ⎤
	// ⎢                                                        ⎥
	// ⎢0  0  0  0      0          0          0           0     ⎥
	// ⎢                                                        ⎥
	// ⎢0  0  0  0      0          0          0           0     ⎥
	// ⎢                                                        ⎥
	// ⎢0  0  0  0      0          0          0           0     ⎥
	// ⎢                                                        ⎥
	// ⎢                              2           3           4 ⎥
	// ⎢0  0  0  0   1152⋅tf   2880⋅tf     5760⋅tf    10080⋅tf  ⎥
	// ⎢                                                        ⎥
	// ⎢                   2          3           4           5 ⎥
	// ⎢0  0  0  0  2880⋅tf    9600⋅tf    21600⋅tf    40320⋅tf  ⎥
	// ⎢                                                        ⎥
	// ⎢                   3           4          5            6⎥
	// ⎢0  0  0  0  5760⋅tf    21600⋅tf   51840⋅tf    100800⋅tf ⎥
	// ⎢                                                        ⎥
	// ⎢                    4          5           6           7⎥
	// ⎣0  0  0  0  10080⋅tf   40320⋅tf   100800⋅tf   201600⋅tf ⎦
}

void MinSnapTraj::GenerateConstraints(const Eigen::MatrixXd &time_powers, Eigen::SparseMatrix<double> &A, Eigen::VectorXd &b_x, Eigen::VectorXd &b_y, Eigen::VectorXd &b_z)
{
	// st A * p = b
	// p is the coefficients
	// A contains endpoints settings the waypoints, matching position  and derivatives
	// match derivatives up to snap at intermediate segments
	// set endpoints to proper location
	//		2 * seg count
	// set endpoint snap jerk acc and velocity to 0 at ends
	//		4 * 2 endpoints
	// set continuity at each internal connection
	//		4 * internal connection count (num waypoints - 2)
	
	assert(A.cols() == kCoeffCount * num_segments_);
	
	assert(A.rows() == 2 * num_segments_ + 8 + 4 * num_internal_joints_);
	std::cout << "A rows: " << A.rows() << " A cols: " << A.cols() << std::endl;
	assert(b_x.size() == 2 * num_segments_ + 8 + 4 * num_internal_joints_);
	assert(b_y.size() == 2 * num_segments_ + 8 + 4 * num_internal_joints_);
	assert(b_z.size() == 2 * num_segments_ + 8 + 4 * num_internal_joints_);
	std::cout << "times" << std::endl;
	std::cout << time_powers;
	std::cout << "asserts succeeded" << std::endl;
	std::cout << "zeroing all" << std::endl;
	Eigen::MatrixXd DenseA = Eigen::MatrixXd::Zero(A.rows(), A.cols());
	b_x = Eigen::VectorXd::Zero(b_x.rows());
	b_y = Eigen::VectorXd::Zero(b_y.rows());
	b_z = Eigen::VectorXd::Zero(b_z.rows());
	Eigen::VectorXd ZeroTimes = Eigen::VectorXd::Zero(kCoeffCount);
	std::cout << "doing math" << std::endl;
	int constraint_count = 0; // Each row in A is a constraint. If we just do each constraint 1 by 1 and increment this we won't get lost.
	
	std::cout << "way point constraints" << std::endl;
	// waypoints first
	for (int segment = 0; segment < num_segments_; ++segment)
	{
		// start time
		DenseA(constraint_count, kCoeffCount * segment) = 1;
		b_x(constraint_count) = waypoints_[segment].pos(0);
		b_y(constraint_count) = waypoints_[segment].pos(1);
		b_z(constraint_count) = waypoints_[segment].pos(2);
		++constraint_count;
		// end time
		Eigen::RowVectorXd constraint_row(kCoeffCount);
		std::cout << "Calc derivative multipliers" << std::endl;
		CalculatePolyDerivativeMultipliers(kCoeffCount, 0, time_powers.row(segment), constraint_row);

		DenseA.block(constraint_count, kCoeffCount * segment, 1, 8) = constraint_row;
		b_x(constraint_count) = waypoints_[segment + 1].pos(0);
		b_y(constraint_count) = waypoints_[segment + 1].pos(1);
		b_z(constraint_count) = waypoints_[segment + 1].pos(2);
		++constraint_count;
	}

	std::cout << "start point derivatives" << std::endl;
	// start point dervatives. No setting of target velocity and acceleration yet. In the future start could be the current one.
	for (int derivative = 1; derivative <= kPosMinDerivative; ++derivative)
	{
		DenseA(constraint_count, derivative) = 1;
		++constraint_count;
	}
	std::cout << "end point derivatives" << std::endl;
	// end point derivatives. No setting of target velocity and acceleration yet. In the future end could a desired velocity
	for (int derivative = 1; derivative <= kPosMinDerivative; ++derivative)
	{
		Eigen::RowVectorXd constraint_row(kCoeffCount);
		CalculatePolyDerivativeMultipliers(kCoeffCount, derivative, time_powers.row(num_segments_ - 1), constraint_row);
		DenseA.block(constraint_count, kCoeffCount * (num_segments_ - 1), 1, 8) = constraint_row;
		++constraint_count;
	}

	std::cout << "continuities of derivatives" << std::endl;
	// Derivative continuities
	for (int joint = 0; joint < num_internal_joints_; ++joint)
	{
		// continuity between segment = joint and segment = joint + 1
		// zeroth derivative of
		// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
		// std::cout << DenseA.format(CleanFmt) << std::endl;
		for (int derivative = 1; derivative <= kPosMinDerivative; ++derivative)
		{
			// end of first segment
			Eigen::RowVectorXd constraint_row(kCoeffCount);
			CalculatePolyDerivativeMultipliers(kCoeffCount, derivative, time_powers.row(joint), constraint_row);
			DenseA.block(constraint_count, kCoeffCount * joint, 1, 8) = constraint_row;
			// start of second segment
			DenseA(constraint_count, kCoeffCount * (joint + 1) + derivative) = -CalculatePolyCoeffMultiplier(derivative, derivative);
			++constraint_count;
		}
	}

	std::cout << "Convert to sparse" << std::endl;
	// convert to sparse (not sure if useful at this point)
	A = DenseA.sparseView();
}

int MinSnapTraj::CalculatePolyCoeffMultiplier(int coeff, int derivative_count)
{
	int multiplier = 1;
	for (int derivative = 0; derivative < derivative_count; ++derivative)
	{
		multiplier *= coeff;
		if (multiplier == 0)
		{
			break;
		}
		--coeff;
	}// coeff is the power that we need to raise T by.
	return multiplier;
}

int MinSnapTraj::CalculatePolyCoeffPower(int coeff, int derivative_count)
{
	int power_val = coeff - derivative_count;
	if (power_val < 0)
	{
		power_val = 0;
	}
	return power_val;
}

void MinSnapTraj::CalculatePolyDerivativeMultipliers(const int coeff_count, const int derivative_count, const Eigen::VectorXd &time_powers_row, Eigen::RowVectorXd &polynomial_derivative)
{
	assert(polynomial_derivative.cols() == coeff_count);
	for (int coeff = 0; coeff < coeff_count; ++coeff)
	{
		polynomial_derivative(coeff) = CalculatePolyCoeffMultiplier(coeff, derivative_count) * time_powers_row(CalculatePolyCoeffPower(coeff, derivative_count));
	}
}

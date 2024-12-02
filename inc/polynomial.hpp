#pragma once
#include <Eigen/Eigen>
#include <iostream>

class Polynomial
{
public:
	Polynomial(Eigen::VectorXd coefficients);
	double Evaluate(double time);
	Polynomial Derivative();
	double coeff(int coeff) const
	{
		assert(coeff < coeff_count_ && coeff >= 0);
		return coefficients_(coeff);
	}
	int coeff_count() const { return coeff_count_; }
	friend std::ostream& operator<<(std::ostream& os, const Polynomial& poly);
private:
	int coeff_count_;
	Eigen::VectorXd coefficients_; // 0th first Nth last
};



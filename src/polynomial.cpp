#include "polynomial.hpp"

Polynomial::Polynomial(Eigen::VectorXd coefficients):coefficients_(coefficients)
{
	coeff_count_ = coefficients_.size();
}

double Polynomial::Evaluate(double time)
{
	// get all the time powers
	Eigen::VectorXd time_powers = Eigen::VectorXd::Ones(coeff_count_);
	time_powers(1) = time;
	for (int power = 2; power < coeff_count_; ++power)
	{
		time_powers(power) = time_powers(power - 1) * time;
	}

	return coefficients_.dot(time_powers);
}

Polynomial Polynomial::Derivative()
{
	Eigen::VectorXd coefficients = coefficients_;
	for (int coeff = 0; coeff < coeff_count_; ++coeff)
	{
		coefficients(coeff) = coefficients(coeff) * coeff;
	}
	return Polynomial(coefficients.tail(coeff_count_ - 1));
}

std::ostream& operator<<(std::ostream& os, const Polynomial& poly)
{
	os << poly.coeff(0);
	for (int coeff = 1; coeff < poly.coeff_count(); ++coeff)
	{
		os << " + " << poly.coeff(coeff) << "t^" << coeff;
	}
    return os;
}
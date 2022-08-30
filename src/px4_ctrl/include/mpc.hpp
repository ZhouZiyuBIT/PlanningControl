#pragma once

#include <casadi/casadi.hpp>

#include <string>

class QuadrotorMPC
{
public:
	QuadrotorMPC(std::string& path, double T, double dt);

	std::vector<double> solve(std::vector<double>& path);

	void set_x0(std::vector<double>& x0);
	void set_lbx_ubx(std::vector<double>& lbx, std::vector<double>& ubx);
	void set_lbg_ubg(std::vector<double>& lbg, std::vector<double>& ubg);

private:
	double _T;
	double _dt;
	int _N;

	casadi::Function solver;
	casadi::Dict _ipopt_option;
	casadi::DM _x0;
	casadi::DM _lbx;
	casadi::DM _ubx;
	casadi::DM _lbg;
	casadi::DM _ubg;
};
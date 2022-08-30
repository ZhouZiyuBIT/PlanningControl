#include "mpc.hpp"

void QuadrotorMPC::set_x0(std::vector<double>& x0)
{
	_x0 = x0;
}

void QuadrotorMPC::set_lbx_ubx(std::vector<double>& lbx, std::vector<double>& ubx)
{
	_lbx = lbx;
	_ubx = ubx;
}

void QuadrotorMPC::set_lbg_ubg(std::vector<double>& lbg, std::vector<double>& ubg)
{
	_lbg = lbg;
	_ubg = ubg;
}

QuadrotorMPC::QuadrotorMPC(std::string& path, double T, double dt)
{
	_T = T;
	_dt = dt;
	_N = int(_T/_dt);

	_ipopt_option["verbose"] = false;
	_ipopt_option["ipopt.tol"] = 1e-4f;
	_ipopt_option["ipopt.max_iter"] = 1000;
	_ipopt_option["ipopt.warm_start_init_point"] = "yes";
	_ipopt_option["ipopt.print_level"] = 0;
	_ipopt_option["print_time"] = false;

	solver = casadi::nlpsol("mpcsolver", "ipopt", path, _ipopt_option);
}

std::vector<double> QuadrotorMPC::solve(std::vector<double>& _path)
{
	casadi::DM path = _path;
	casadi::DMDict res = solver(casadi::DMDict{
		                       {"x0",  _x0  },
				       {"lbx", _lbx },
				       {"ubx", _ubx },
				       {"p",   path },
				       {"lbg", _lbg },
				       {"ubg", _ubg }
				       });
	_x0 = res["x"];

	std::vector<double> u_opt(_x0(casadi::Slice(10,14)));

	return u_opt;
}

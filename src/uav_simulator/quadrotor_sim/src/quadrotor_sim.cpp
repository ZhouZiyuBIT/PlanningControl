#include"quadrotor_sim.hpp"

#include<yaml-cpp/yaml.h>

#include<cmath>
#include<string>
#include<vector>
#include<iostream>

void q2R(float q[4], float R[3][3])
{
    float q00 = q[0]*q[0];
    float q01 = q[0]*q[1];
    float q02 = q[0]*q[2];
    float q03 = q[0]*q[3];
    float q11 = q[1]*q[1];
    float q12 = q[1]*q[2];
    float q13 = q[1]*q[3];
    float q22 = q[2]*q[2];
    float q23 = q[2]*q[3];
    float q33 = q[3]*q[3];

    R[0][0] = q00+q11-q22-q33;
    R[0][1] = -2*q03+2*q12;
    R[0][2] = 2*q02+2*q13;
    R[1][0] = 2*q03+2*q12;
    R[1][1] = q00-q11+q22-q33;
    R[1][2] = -2*q01+2*q23;
    R[2][0] = -2*q02+2*q13;
    R[2][1] = 2*q01+2*q23;
    R[2][2] = q00-q11-q22+q33;
}

void Rv(float R[3][3], float v[3], float Rv[3])
{
    Rv[0] = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    Rv[1] = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    Rv[2] = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
}

void R_v(float R[3][3], float v[3], float R_v[3])
{
    R_v[0] = R[0][0]*v[0] + R[1][0]*v[1] + R[2][0]*v[2];
    R_v[1] = R[0][1]*v[0] + R[1][1]*v[1] + R[2][1]*v[2];
    R_v[2] = R[0][2]*v[0] + R[1][2]*v[1] + R[2][2]*v[2];
}


PID::PID()
{
    _e_last = 0;
    _init = 0;
}

void PID::setParameters(float P, float I, float D, float I_lim, float Out_lim)
{
    _P = P;
    _I = I;
    _D = D;
    _I_lim = I_lim;
    _Out_lim = Out_lim;
}

float PID::update(float e)
{
    _init += e;
    if(_init>_I_lim) _init = _I_lim;
    else if(_init< -_I_lim) _init = -_I_lim;

    float out = _P*e + _I*_init + _D*(e-_e_last);
    _e_last = e;
    if(out > _Out_lim) out = _Out_lim;
    else if(out < -_Out_lim) out = -_Out_lim;

    return out;
}


AzControl::AzControl()
{
    _e_last = 0;
    _init = 0;
}

void AzControl::setParameters(float P, float I, float D, float I_lim, float Out_lim)
{
    _P = P;
    _I = I;
    _D = D;
    _I_lim = I_lim;
    _Out_lim = Out_lim;
}

float AzControl::update(float e, float dt)
{
    _init += e;
    if(_init*dt>_I_lim) _init = _I_lim/dt;
    else if(_init*dt< -_I_lim) _init = -_I_lim/dt;

    float out = _P*e + _I*_init*dt + _D*(e-_e_last)/dt;
    _e_last = e;
    if(out > 0) out = 0;
    else if(out < -_Out_lim) out = -_Out_lim;

    return out;
}

WControl::WControl()
{
    _e_last[0] = 0,_e_last[1] = 0,_e_last[2] = 0;
    _init[0] = 0,_init[1] = 0,_init[2] = 0;
}

void WControl::setParameters(std::vector<float> P, std::vector<float> I, std::vector<float> D, std::vector<float> I_lim, std::vector<float> Out_lim)
{
    _P[0] = P[0],_P[1] = P[1],_P[2] = P[2];
    _I[0] = I[0],_I[1] = I[1],_I[2] = I[2];
    _D[0] = D[0],_D[1] = D[1],_D[2] = D[2];
    _I_lim[0] = I_lim[0],_I_lim[1] = I_lim[1],_I_lim[2] = I_lim[2];
    _Out_lim[0] = Out_lim[0],_Out_lim[1] = Out_lim[1],_Out_lim[2] = Out_lim[2];
}

void WControl::update(float e[3], float dt, float out[3])
{
    _init[0] += e[0],_init[1] += e[1],_init[2] += e[2];
    if(_init[0]*dt>_I_lim[0]) _init[0] = _I_lim[0]/dt;
    else if(_init[0]*dt< -_I_lim[0]) _init[0] = -_I_lim[0]/dt;
    if(_init[1]*dt>_I_lim[1]) _init[1] = _I_lim[1]/dt;
    else if(_init[1]*dt< -_I_lim[1]) _init[1] = -_I_lim[1]/dt;
    if(_init[2]*dt>_I_lim[2]) _init[2] = _I_lim[2]/dt;
    else if(_init[2]*dt< -_I_lim[2]) _init[2] = -_I_lim[2]/dt;

    out[0] = _P[0]*e[0] + _I[0]*_init[0]*dt + _D[0]*(e[0]-_e_last[0])/dt;
    _e_last[0] = e[0];
    if(out[0] > _Out_lim[0]) out[0] = _Out_lim[0];
    else if(out[0] < -_Out_lim[0]) out[0] = -_Out_lim[0];

    out[1] = _P[1]*e[1] + _I[1]*_init[1]*dt + _D[1]*(e[1]-_e_last[1])/dt;
    _e_last[1] = e[1];
    if(out[1] > _Out_lim[1]) out[1] = _Out_lim[1];
    else if(out[1] < -_Out_lim[1]) out[1] = -_Out_lim[1];

    out[2] = _P[2]*e[2] + _I[2]*_init[2]*dt + _D[2]*(e[2]-_e_last[2])/dt;
    _e_last[2] = e[2];
    if(out[2] > _Out_lim[2]) out[2] = _Out_lim[2];
    else if(out[2] < -_Out_lim[2]) out[2] = -_Out_lim[2];
}

FirstOrderLPF::FirstOrderLPF(double dt, double tau)
{
    _dt = dt;
    _tau = tau;
    _y_last = 0;
}

void FirstOrderLPF::setDt(double dt)
{
    _dt = dt;
}

void FirstOrderLPF::setTimeConstant(double tau)
{
    _tau = tau;
}

double FirstOrderLPF::update(double x)
{
    double pi = 3.1415927;
    double alpha = 2*pi*_dt/(2*pi*_dt+_tau);
    double y = alpha*x + (1-alpha)*_y_last;
    _y_last = y;
        
    return y;
}


QuadrotorSimulation::QuadrotorSimulation(std::string& config)
{
    YAML::Node cfg = YAML::LoadFile(config);
    t_sim = 0.f;
    t_step = cfg["t_step"].as<float>();
    dt_lowpid = cfg["dt_lowpid"].as<float>();
    dt_control = cfg["dt_control"].as<float>();

    // init
    _p[0] = 0,_p[1]=0,_p[2]=0;
    _v[0] = 0,_v[1]=0,_v[2]=0;
    _q[0] = 1,_q[1]=0,_q[2]=0,_q[3]=0;
    q2R(_q, _R);
    _a[0] = 0,_a[1]=0,_a[2]=0;
    _a_B[0] = 0,_a_B[1]=0,_a_B[2]=0;
    _w_B[0] = 0,_w_B[1]=0,_w_B[2]=0;
    _dw_B[0] = 0,_dw_B[1]=0,_dw_B[2]=0;
    
    _f_d[0]=0,_f_d[1]=0,_f_d[2]=0;
    _tau_d[0]=0,_tau_d[1]=0,_tau_d[2]=0;

    // parameters
    _G = cfg["G"].as<float>();
    _mass = cfg["mass"].as<float>();
    _J[0] = cfg["Jxx"].as<float>(),_J[1]=cfg["Jyy"].as<float>(),_J[2]=cfg["Jzz"].as<float>();

    _az_control.setParameters(cfg["az_P"].as<float>(),cfg["az_I"].as<float>(),cfg["az_D"].as<float>(),cfg["f_lim"].as<float>(),cfg["f_lim"].as<float>());
    _w_control.setParameters(cfg["w_P"].as<std::vector<float>>(),
                             cfg["w_I"].as<std::vector<float>>(),
                             cfg["w_D"].as<std::vector<float>>(),
                             cfg["w_lim"].as<std::vector<float>>(),
                             cfg["w_lim"].as<std::vector<float>>());

    _az_lpf.setTimeConstant(cfg["az_lpf"].as<float>());
    _wx_lpf.setTimeConstant(cfg["wx_lpf"].as<float>());
    _wy_lpf.setTimeConstant(cfg["wy_lpf"].as<float>());
    _wz_lpf.setTimeConstant(cfg["wz_lpf"].as<float>());
}

void QuadrotorSimulation::set_position(float pos[3])
{
    _p[0] = pos[0];
    _p[1] = pos[1];
    _p[2] = pos[2];
}

std::vector<float> QuadrotorSimulation::get_position()
{
    std::vector<float> p;
    p.push_back(_p[0]);
    p.push_back(_p[1]);
    p.push_back(_p[2]);
    return p;
}

std::vector<float> QuadrotorSimulation::get_velocity()
{
    std::vector<float> v;
    v.push_back(_v[0]);
    v.push_back(_v[1]);
    v.push_back(_v[2]);
    return v;
}

std::vector<float> QuadrotorSimulation::get_quaternion()
{
    std::vector<float> q;
    q.push_back(_q[0]);
    q.push_back(_q[1]);
    q.push_back(_q[2]);
    q.push_back(_q[3]);
    return q;
}

std::vector<std::vector<float>> QuadrotorSimulation::get_R()
{
    std::vector<std::vector<float>> R;
    std::vector<float> Rx, Ry, Rz;
    Rx.push_back(_R[0][0]),Rx.push_back(_R[1][0]),Rx.push_back(_R[2][0]);
    Ry.push_back(_R[0][1]),Ry.push_back(_R[1][1]),Ry.push_back(_R[2][1]);
    Rz.push_back(_R[0][2]),Rz.push_back(_R[1][2]),Rz.push_back(_R[2][2]);
    R.push_back(Rx);
    R.push_back(Ry);
    R.push_back(Rz);
    return R;
}

std::vector<float> QuadrotorSimulation::get_w_B()
{
    std::vector<float> w_B;
    w_B.push_back(_w_B[0]);
    w_B.push_back(_w_B[1]);
    w_B.push_back(_w_B[2]);
    return w_B;
}

std::vector<float> QuadrotorSimulation::get_a_B()
{
    std::vector<float> a_B;
    a_B.push_back(_a_B[0]);
    a_B.push_back(_a_B[1]);
    a_B.push_back(_a_B[2]);
    return a_B;
}

std::vector<float> QuadrotorSimulation::get_f_ext()
{
    float f_[3];
    Rv(_R, _f_d, f_);
    std::vector<float> f_ext;
    f_ext.push_back(f_[0]);
    f_ext.push_back(f_[1]);
    f_ext.push_back(f_[2]);
    return f_ext;
}

void QuadrotorSimulation::control(float az_s, float w_s[3])
{
    float tt = t_sim + dt_control;
    while(t_sim<tt)
    {
        float f;
        float tau[3];
            
        // az control
        float az_e = az_s-_a_B[2];
        f = _az_control.update(az_e, dt_lowpid);

        // w control
        float w_e[3];
        w_e[0] = w_s[0]-_w_B[0];
        w_e[1] = w_s[1]-_w_B[1];
        w_e[2] = w_s[2]-_w_B[2];
        _w_control.update(w_e, dt_lowpid, tau);

        // step
        float ttt = t_sim + dt_lowpid;
        while(t_sim<ttt)
        {
            f = (double)_az_lpf.update(f);
            tau[0] = (double)_wx_lpf.update(tau[0]);
            tau[1] = (double)_wy_lpf.update(tau[1]);
            tau[2] = (double)_wz_lpf.update(tau[2]);
            _step(f, tau);
        }
    }
}

void QuadrotorSimulation::_disturbance()
{
    float f_b[3];

    // wind
    float f_wind[3] = {0, 0, 0};
    R_v(_R, f_wind, f_b);
    _f_d[0] = f_b[0];
    _f_d[1] = f_b[1];
    _f_d[2] = f_b[2];

    // v resistance
    float v_B[3];
    R_v(_R, _v, v_B);
    f_b[0] = v_B[0]*1;
    f_b[1] = v_B[1]*1;
    f_b[2] = v_B[2]*2;

    _f_d[0] += f_b[0];
    _f_d[1] += f_b[1];
    _f_d[2] += f_b[2];
}

void QuadrotorSimulation::_step(float f, float tau[3])
{
    _disturbance();
    // p
    _p[0] += _v[0]*t_step;
    _p[1] += _v[1]*t_step;
    _p[2] += _v[2]*t_step;

    // v a a_B
    _a_B[0] = (_f_d[0])/_mass;
    _a_B[1] = (_f_d[1])/_mass;
    _a_B[2] = (_f_d[2]+f)/_mass;
    Rv(_R,_a_B, _a);
    _a[2] += _G;
    _v[0] += _a[0]*t_step;
    _v[1] += _a[1]*t_step;
    _v[2] += _a[2]*t_step;

    // _q _R
    float dq[4];
    dq[0] = 0.5f*(-_q[1]*_w_B[0] - _q[2]*_w_B[1] - _q[3]*_w_B[2]);
    dq[1] = 0.5f*( _q[0]*_w_B[0] - _q[3]*_w_B[1] + _q[2]*_w_B[2]);
    dq[2] = 0.5f*( _q[3]*_w_B[0] + _q[0]*_w_B[1] - _q[1]*_w_B[2]);
    dq[3] = 0.5f*(-_q[2]*_w_B[0] + _q[1]*_w_B[1] + _q[0]*_w_B[2]);
    _q[0] += dq[0]*t_step;
    _q[1] += dq[1]*t_step;
    _q[2] += dq[2]*t_step;
    _q[3] += dq[3]*t_step;
    float q_len = (float)std::sqrt(_q[0]*_q[0] + _q[1]*_q[1] + _q[2]*_q[2] + _q[3]*_q[3]);
    _q[0] /= q_len;
    _q[1] /= q_len;
    _q[2] /= q_len;
    _q[3] /= q_len;
    q2R(_q, _R);

    // _w_B _dw_B
    _dw_B[0] = ((_J[1]-_J[2])*_w_B[1]*_w_B[2] + tau[0] + _tau_d[0])/_J[0];
    _dw_B[1] = ((_J[2]-_J[0])*_w_B[0]*_w_B[2] + tau[1] + _tau_d[1])/_J[1];
    _dw_B[2] = ((_J[0]-_J[1])*_w_B[0]*_w_B[1] + tau[2] + _tau_d[2])/_J[2];
    _w_B[0] += _dw_B[0]*t_step;
    _w_B[1] += _dw_B[1]*t_step;
    _w_B[2] += _dw_B[2]*t_step;

    // time
    t_sim += t_step;
}


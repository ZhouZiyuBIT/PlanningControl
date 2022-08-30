#pragma once

#include<string>
#include<vector>

class PID
{
    public:
    PID();
    void setParameters(float P, float I, float D, float I_lim, float Out_lim);

    float update(float e);

    private:
    float _P, _I, _D;
    float _I_lim, _Out_lim;
    float _e_last;
    float _init;
};

class AzControl
{
    public:
    AzControl();

    void setParameters(float P, float I, float D, float I_lim, float Out_lim);

    float update(float e, float dt);

    private:
    float _P, _I, _D;
    float _I_lim, _Out_lim;
    float _e_last;
    float _init;
};

class WControl
{
    public:
    WControl();

    void setParameters(std::vector<float> P, std::vector<float> I, std::vector<float> D, std::vector<float> I_lim, std::vector<float> Out_lim);

    void update(float e[3], float dt, float out[3]);

    private:
    float _P[3], _I[3], _D[3];
    float _I_lim[3], _Out_lim[3];
    float _e_last[3];
    float _init[3];
};

class FirstOrderLPF
{
    public:
    FirstOrderLPF(double dt=0.001, double tau=0.005);

    void setDt(double dt);

    void setTimeConstant(double tau);

    double update(double x);

    private:
    double _y_last;
    double _tau;
    double _dt;
};

class QuadrotorSimulation
{
    public:
    QuadrotorSimulation(std::string& config);

    void set_position(float pos[3]);

    void control(float az_s, float w_s0, float w_s1, float w_s2);

    std::vector<float> get_position();

    std::vector<float> get_velocity();

    std::vector<float> get_quaternion();

    std::vector<std::vector<float>> get_R();

    std::vector<float> get_w_B();

    std::vector<float> get_a_B();

    std::vector<float> get_f_ext();

    void control(float az_s, float w_s[3]);

    private:
    
    // states
    float _p[3];
    float _v[3];
    float _q[4];
    float _R[3][3];
    float _a[3];
    float _a_B[3];
    float _w_B[3];
    float _dw_B[3];

    // disturbances
    float _f_d[3];
    float _tau_d[3];

    // time
    float t_sim;
    float t_step;
    float dt_lowpid;
    float dt_control;

    // parameters
    float _G;
    float _mass;
    float _J[3];

    // lowpids
    AzControl _az_control;
    WControl _w_control;

    // low pass filter
    FirstOrderLPF _az_lpf;
    FirstOrderLPF _wx_lpf;
    FirstOrderLPF _wy_lpf;
    FirstOrderLPF _wz_lpf;

    void _disturbance();

    void _step(float f, float tau[3]);
};

#pragma once

// OptiTrack SDK include
#include"NatNetTypes.h"
#include"NatNetCAPI.h"
#include"NatNetClient.h"

#include<iostream>
#include<functional>
#include<pthread.h>
#include<mutex>

// #include<ctime>
#include<sys/time.h>

#include<sys/socket.h>
#include<arpa/inet.h>

class TicToc
{
    public:
    TicToc()
    {
        gettimeofday(&begin, NULL);
        gettimeofday(&end, NULL);
    }
    void Tic()
    {
        gettimeofday(&begin, NULL);
    }
    void Toc()
    {
        gettimeofday(&end, NULL);
    }
    void print(const char * str)
    {
        std::cout << str << 1000000*(end.tv_sec - begin.tv_sec) + end.tv_usec - begin.tv_usec << std::endl;
    }
    private:
    struct timeval begin, end;
};

class Px4Bridge
{
    using state_callback_f = std::function<void(float state[10])>;
    using sensor_imu_callback_f = std::function<void(float w[3], float a[3])>;
public:
    Px4Bridge(){}
    ~Px4Bridge();

    int setup_port(const char* port);
    void setup_optitrack(const char *server_ip);
    int add_fordwarding(const char* local_addr, int local_port, const char* remote_addr, int remote_port);
    int core_start();

    void registe_rcv_state_callback(const state_callback_f &handler);
    void registe_rcv_sensor_imu_callback(const sensor_imu_callback_f &handler);
    int send_control_u(float thrust_sp, float wx_sp, float wy_sp, float wz_sp);
    void send_odom_data(float pos_x, float pos_y, float pos_z,
                         float qw, float qx, float qy, float qz);

    int set_thread_rt(int priority);

private:
    int32_t _px4_rigidbody_id = 5;

    state_callback_f _rcv_state_handler = nullptr;
    sensor_imu_callback_f _rcv_sensor_imu_handler = nullptr;

    TicToc tim;
    /*
    px4 mavlink
    */
    int _mavlink_port_fd = -1;
    uint8_t _mavlink_rcv_buffer[2048];
    std::mutex _port_io_mutex;

    bool _fordwarding_flag = false;
    int _fordwarding_udp_fd = -1;
    struct sockaddr_in _remote_addr {};

    static void* core_run(void *args){((Px4Bridge*)args)->_core_run(); }
    void _core_run();
    pthread_t _core_thread = -1;

    /*
    OptiTrack MotionCap
    */
   NatNetClient *_pClient = NULL;
   sNatNetClientConnectParams _connectParams;
   sServerDescription _serverDescription;

   static void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);
   static void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);

};

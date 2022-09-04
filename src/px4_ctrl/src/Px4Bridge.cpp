#include"Px4Bridge.hpp"

#include<iostream>
#include<iomanip>
#include<string>

// #include <stdlib.h>
#include<fcntl.h>
#include<termios.h> //set baud rate
#include <unistd.h>
#include<string.h>
#include<poll.h>

#include"mavlink/v2.0/standard/mavlink.h"
#include"mavlink/v2.0/mavlink_helpers.h"

long get_time_us()
{
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    return (time_now.tv_sec*1000000 + time_now.tv_usec);
}

Px4Bridge::~Px4Bridge()
{
    close(_mavlink_port_fd);
    if(_pClient)
    {
        _pClient->Disconnect();
        delete _pClient;
        _pClient = NULL;
    }
}

void Px4Bridge::setup_optitrack(const char *server_ip)
{
    unsigned char version[4];
    NatNet_GetVersion(version);
    std::cout << "NatNet Client Version:" << (int)version[0] << "." << (int)version[1] << "." 
                                          << (int)version[2] << "." << (int)version[3] << std::endl;
    
    NatNet_SetLogCallback(MessageHandler);

    _pClient = new NatNetClient();
    _pClient->SetFrameReceivedCallback(DataHandler, this);

    _connectParams.connectionType = ConnectionType_Unicast;
    _connectParams.serverAddress = server_ip;
    
    _pClient->Disconnect();
    int retCode = _pClient->Connect(_connectParams);
    if(retCode != ErrorCode_OK)
    {
        std::cout << "Unable to connect to server, error code: " << retCode << std::endl;
        exit(-1);
    }
    memset( &_serverDescription, 0, sizeof( _serverDescription ) );
    int ret = _pClient->GetServerDescription( &_serverDescription );
    if ( ret != ErrorCode_OK || ! _serverDescription.HostPresent )
    {
        std::cout << "Unable to connect to server. Host not present. Exiting." << std::endl;
    }
}

int Px4Bridge::add_fordwarding(const char* local_addr, int local_port, const char* remote_addr, int remote_port)
{
    _fordwarding_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(_fordwarding_udp_fd < 0)
    {
        std::cout << "create socket error" << std::endl;
        return -1;
    }
    int flag = fcntl(_fordwarding_udp_fd, F_GETFL, 0);
    flag |= O_NONBLOCK;
    if(fcntl(_fordwarding_udp_fd, F_SETFL, flag) < 0)
    {
        std::cout << "set socket nonblock error" << std::endl;
        return -1;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, local_addr, &(addr.sin_addr));
    addr.sin_port = htons(local_port);
    if(bind(_fordwarding_udp_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        std::cout << "socket bind error" << std::endl;
        return -1;
    }

    _remote_addr.sin_family = AF_INET;
    inet_pton(AF_INET, remote_addr, &(_remote_addr.sin_addr));
    _remote_addr.sin_port = htons(remote_port);

    _fordwarding_flag = true;

    return _fordwarding_udp_fd;
}

void NATNET_CALLCONV Px4Bridge::MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    std::cout << "[NatNetLib]";

    switch ( msgType )
    {
        case Verbosity_Debug:
            std::cout << "  [DEBUG]:";
            break;
        case Verbosity_Info:
            std::cout << "  [INFO]:";
            break;
        case Verbosity_Warning:
            std::cout << "  [WARN]:";
            break;
        case Verbosity_Error:
            std::cout << "  [ERROR]:";
            break;
        default:
            std::cout << " [?????]:";
            break;
    }

    std::cout << msg << std::endl;
}

void NATNET_CALLCONV Px4Bridge::DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    Px4Bridge *ppx4bridge = (Px4Bridge*) pUserData;
    NatNetClient* pClient = ppx4bridge->_pClient;


	// Rigid Bodies
//	printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	for(int i=0; i < data->nRigidBodies; i++)
	{
        if(data->RigidBodies[i].ID == ppx4bridge->_px4_rigidbody_id)
        {
            ppx4bridge->send_odom_data(-data->RigidBodies[i].y, -data->RigidBodies[i].x, -data->RigidBodies[i].z,
                                        data->RigidBodies[i].qw,
                                        -data->RigidBodies[i].qy, -data->RigidBodies[i].qx, -data->RigidBodies[i].qz);
        }

        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
//      bool bTrackingValid = data->RigidBodies[i].params & 0x01;

//		printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		// printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		// printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
		// 	data->RigidBodies[i].x,
		// 	data->RigidBodies[i].y,
		// 	data->RigidBodies[i].z,
		// 	data->RigidBodies[i].qx,
		// 	data->RigidBodies[i].qy,
		// 	data->RigidBodies[i].qz,
		// 	data->RigidBodies[i].qw);
	}
}

void Px4Bridge::send_odom_data(float pos_x, float pos_y, float pos_z,
                     float qw, float qx, float qy, float qz)
{
    uint64_t usec = get_time_us();
    uint8_t send_buf[512];
    mavlink_message_t msg;
    float pose_covariance[21], vel_covariance[21];
    float q[4];

    q[0] = qw;
    q[1] = qx;
    q[2] = qy;
    q[3] = qz;

    pose_covariance[0] = NAN;  //
    vel_covariance[0] = NAN;

    mavlink_msg_odometry_pack(0, 0, &msg, usec,
                              MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD,
                              pos_x, pos_y, pos_z,
                              q,
                              NAN, NAN, NAN, NAN, NAN, NAN,
                              pose_covariance, vel_covariance,
                              0, MAV_ESTIMATOR_TYPE_VISION);
                              
    uint16_t send_len = mavlink_msg_to_send_buffer(send_buf, &msg);
    _port_io_mutex.lock();
    write(_mavlink_port_fd, send_buf, send_len);
    _port_io_mutex.unlock();
}

int Px4Bridge::send_control_u(float thrust_sp, float wx_sp, float wy_sp, float wz_sp)
{
    uint8_t send_buf[512];
    mavlink_message_t msg;
    mavlink_msg_quadrotor_control_input_pack(0, 0, &msg,
                                             thrust_sp,
                                             wx_sp, wy_sp, wz_sp);

    uint16_t send_len = mavlink_msg_to_send_buffer(send_buf, &msg);
    _port_io_mutex.lock();
    write(_mavlink_port_fd, send_buf, send_len);
    _port_io_mutex.unlock();

    return 1;
}

int Px4Bridge::setup_port(const char* port)
{
    // open() hangs on macOS or Linux devices(e.g. pocket beagle) unless you give it O_NONBLOCK
    _mavlink_port_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_mavlink_port_fd == -1) {
        std::cout << "serial port open failed: " << std::endl;
        return -1;
    }
    // We need to clear the O_NONBLOCK again because we can block while reading
    // as we do it in a separate thread.
    if (fcntl(_mavlink_port_fd, F_SETFL, 0) == -1) {
        std::cout << "serial port fcntl failed: " << std::endl;
        return -1;
    }
    struct termios tc;
    bzero(&tc, sizeof(tc));

    if (tcgetattr(_mavlink_port_fd, &tc) != 0) {
        std::cout << "tcgetattr failed: " << std::endl;
        close(_mavlink_port_fd);
    }
    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
    tc.c_cflag |= CS8;

    tc.c_cc[VMIN] = 0; // We are ok with 0 bytes.
    tc.c_cc[VTIME] = 10; // Timeout after 1 second.

    // if (_flow_control) {
    //     tc.c_cflag |= CRTSCTS;
    // }
    tc.c_cflag |= CLOCAL; // Without this a write() blocks indefinitely.
    const int baudrate_or_define = B57600;
    if (cfsetispeed(&tc, baudrate_or_define) != 0) {
        std::cout << "cfsetispeed failed: " << std::endl;
        close(_mavlink_port_fd);
        return -1;
    }

    if (cfsetospeed(&tc, baudrate_or_define) != 0) {
        std::cout << "cfsetospeed failed: " << std::endl;
        close(_mavlink_port_fd);
        return -1;
    }

    if (tcsetattr(_mavlink_port_fd, TCSANOW, &tc) != 0) {
        std::cout << "tcsetattr failed: " << std::endl;
        close(_mavlink_port_fd);
        return -1;
    }

    return _mavlink_port_fd;
}

void Px4Bridge::registe_rcv_state_callback(const state_callback_f &handler)
{
    _rcv_state_handler = handler;
}

void Px4Bridge::registe_rcv_sensor_imu_callback(const sensor_imu_callback_f &handler)
{
    _rcv_sensor_imu_handler = handler;
}

void Px4Bridge::_core_run()
{
    set_thread_rt(60);

    unsigned char fordwarding_send_buf[1024];
    unsigned char fordwarding_recv_buf[1024];

    struct pollfd fds[2];
    fds[0].fd = _mavlink_port_fd;
    fds[0].events = POLLIN;
    fds[1].fd = _fordwarding_udp_fd;
    fds[1].events = POLLIN;

    while(1)
    {
        int pollrc = poll(fds, 2, 1000);
        
        if (pollrc == 0 || !(fds[0].revents & POLLIN)) {
            continue;
        } else if (pollrc == -1) {
            std::cout << "read poll failure: " << std::endl;
        }
        else
        {
            mavlink_status_t status;
            mavlink_message_t msg;

            mavlink_attitude_t attitude;
            mavlink_quadrotor_state_t q_state;
            mavlink_sensor_imu_t s_imu;
            mavlink_debug_t dv;

            // We enter here if (fds[0].revents & POLLIN) == true
            _port_io_mutex.lock();
            int recv_len = static_cast<int>(read(_mavlink_port_fd, _mavlink_rcv_buffer, sizeof(_mavlink_rcv_buffer)));
            _port_io_mutex.unlock();
            if (recv_len < -1) {
                std::cout << "serial read failure: " << std::endl;
                continue;
            }
            if (recv_len > static_cast<int>(sizeof(_mavlink_rcv_buffer)) || recv_len == 0) {
                std::cout << "serial read error" << std::endl;
                continue;
            }
            
            if(_fordwarding_flag && (_fordwarding_udp_fd > 0))
            {
                sendto(_fordwarding_udp_fd, _mavlink_rcv_buffer, recv_len,
                       0,
                       reinterpret_cast<const sockaddr*>(&_remote_addr),
                       sizeof(_remote_addr));
            }

            for(int i=0; i<recv_len; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, _mavlink_rcv_buffer[i], &msg, &status))
                {
                    // printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
                    // ... DECODE THE MESSAGE PAYLOAD HERE ...
                    // std::cout << "serial msg id:" << msg.msgid << std::endl;
                    switch(msg.msgid) 
                    {
                        case MAVLINK_MSG_ID_ATTITUDE: // ID for ATTITUDE
                        {
                            // if(_fordwarding_flag && (_fordwarding_udp_fd > 0))
                            // {
                            //     int len = mavlink_msg_to_send_buffer(fordwarding_send_buf, &msg);
                            //     sendto(_fordwarding_udp_fd, fordwarding_send_buf, len,
                            //         0,
                            //         reinterpret_cast<const sockaddr*>(&_remote_addr),
                            //         sizeof(_remote_addr));
                            // }

                            // tim.Toc();
                            // tim.print("att dt:");
                            // tim.Tic();
                            // Get all fields in payload (into attitude)
                            // mavlink_msg_attitude_decode(&msg, &attitude);
                            // std::cout << "roll:" << attitude.roll << "    pitch:" << attitude.pitch << "    yaw:" << attitude.yaw << std::endl;
                        }
                        break;

                        case MAVLINK_MSG_ID_SENSOR_IMU:
                        {
                            mavlink_msg_sensor_imu_decode(&msg, &s_imu);
                            // std::cout << "received imu" << std::endl;
                            if(_rcv_sensor_imu_handler)
                            {
                                float w[3];
                                float a[3];
                                w[0] = s_imu.wx;
                                w[1] = s_imu.wy;
                                w[2] = s_imu.wz;
                                a[0] = s_imu.ax;
                                a[1] = s_imu.ay;
                                a[2] = s_imu.az;
                                _rcv_sensor_imu_handler(w, a);
                            }
                        }
                        break;

                        case MAVLINK_MSG_ID_QUADROTOR_STATE:
                            // tim.Toc();
                            // tim.print("q_state dt:");
                            // tim.Tic();
                            mavlink_msg_quadrotor_state_decode(&msg, &q_state);
                            
                            // tim.Tic();
                            if(_rcv_state_handler)
                            {
                                float q_s[10] = {q_state.Px, q_state.Py, q_state.Pz,
                                                 q_state.Vx, q_state.Vy, q_state.Vz,
                                                 q_state.Qw, q_state.Qx, q_state.Qy, q_state.Qz};
                                _rcv_state_handler(q_s);
                            }
                            // tim.Toc();
                            // tim.print("call used tim:");
                        break;

                        case MAVLINK_MSG_ID_DEBUG:
                            mavlink_msg_debug_decode(&msg, &dv);
                            std::cout << "dv: " << dv.time_boot_ms << "         " << dv.value << std::endl;
                        break;

                        default:
                        break;
                    }
                }
            }
        }

        //UDP fordwarding
        if (pollrc == 0 || !(fds[1].revents & POLLIN)) {
            continue;
        } else if (pollrc == -1) {
            std::cout << "read poll failure: " << std::endl;
        }
        else
        {
            mavlink_status_t status;
            mavlink_message_t msg;

            struct sockaddr_in src_addr{};
            socklen_t sockaddr_len = sizeof(src_addr);
            int udp_rcv_len = recvfrom(_fordwarding_udp_fd, fordwarding_recv_buf, 1024,
                                      0,
                                      reinterpret_cast<struct sockaddr*>(&src_addr),
                                      &sockaddr_len);
            
            if (udp_rcv_len < -1) {
                std::cout << "udp read failure: " << std::endl;
                continue;
            }
            if (udp_rcv_len > static_cast<int>(sizeof(fordwarding_recv_buf)) || udp_rcv_len == 0) {
                std::cout << "udp read error" << std::endl;
                continue;
            }

            // for(int i=0; i<udp_rcv_len; i++)
            // {
            //     if (mavlink_parse_char(MAVLINK_COMM_1, fordwarding_recv_buf[i], &msg, &status))
            //     {
            //         std::cout << "upd msg id:" << msg.msgid << std::endl;
            //     }
            // }
            
            _port_io_mutex.lock();
            write(_mavlink_port_fd, fordwarding_recv_buf, udp_rcv_len);
            _port_io_mutex.unlock();
        }
        
    }
}

int Px4Bridge::core_start()
{
    return pthread_create(&_core_thread, NULL, core_run, this);
}

int Px4Bridge::set_thread_rt(int priority)
{
    pid_t pid = getpid();
    
    struct sched_param s_param;
    s_param.sched_priority = priority;
    int ret;
    if((ret = sched_setscheduler(pid, SCHED_FIFO, &s_param)) < 0)
    {
        std::cout << pid << "thread set scheduler error!" << std::endl;
    }

    return ret;
}

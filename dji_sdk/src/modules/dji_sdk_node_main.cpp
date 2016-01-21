#include <dji_sdk/dji_sdk_node.h>
#include <functional>
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))

//----------------------------------------------------------
// timer spin_function 50Hz
//----------------------------------------------------------
void DJISDKNode::broadcast_callback()
{
    sdk_std_msg_t recv_sdk_std_msgs;
    unsigned short msg_flags;
    DJI_Pro_Get_Broadcast_Data(&recv_sdk_std_msgs, &msg_flags);

    static int frame_id = 0;
    frame_id ++;

    auto current_time = ros::Time::now();

    //update attitude msg
    if ((msg_flags & ENABLE_MSG_Q) && (msg_flags & ENABLE_MSG_W)) {
        attitude_quaternion.header.frame_id = "/world";
        attitude_quaternion.header.stamp = current_time;
        attitude_quaternion.q0 = recv_sdk_std_msgs.q.q0;
        attitude_quaternion.q1 = recv_sdk_std_msgs.q.q1;
        attitude_quaternion.q2 = recv_sdk_std_msgs.q.q2;
        attitude_quaternion.q3 = recv_sdk_std_msgs.q.q3;
        attitude_quaternion.wx = recv_sdk_std_msgs.w.x;
        attitude_quaternion.wy = recv_sdk_std_msgs.w.y;
        attitude_quaternion.wz = recv_sdk_std_msgs.w.z;
        attitude_quaternion.ts = recv_sdk_std_msgs.time_stamp;
        attitude_quaternion_publisher.publish(attitude_quaternion);
        LOG_MSG_STAMP("/dji_sdk/attitude_quaternion", attitude_quaternion, current_time, 2);
    }

    //update global_position msg
    if ((msg_flags & ENABLE_MSG_POS)) {
        global_position.header.frame_id = "/world";
        global_position.header.stamp = current_time;
        global_position.ts = recv_sdk_std_msgs.time_stamp;
        global_position.latitude = recv_sdk_std_msgs.pos.lati * 180.0 / C_PI;
        global_position.longitude = recv_sdk_std_msgs.pos.longti * 180.0 / C_PI;
        global_position.height = recv_sdk_std_msgs.pos.height;
        global_position.altitude = recv_sdk_std_msgs.pos.alti;
        global_position.health = recv_sdk_std_msgs.pos.health_flag;
        global_position_publisher.publish(global_position);
        LOG_MSG_STAMP("/dji_sdk/global_position", global_position, current_time, 1);

        //TODO:
        // FIX BUG about flying at lat = 0
        if (global_position.ts != 0 && global_position_ref_seted == 0 && global_position.latitude != 0) {
            global_position_ref = global_position;
            global_position_ref_seted = 1;
        }

        //update local_position msg
        local_position.header.frame_id = "/world";
        local_position.header.stamp = current_time;
        gps_convert_ned(
                local_position.x,
                local_position.y,
                global_position.longitude,
                global_position.latitude,
                global_position_ref.longitude,
                global_position_ref.latitude
                );
        local_position.z = global_position.height;
        local_position.ts = global_position.ts;
        local_position_ref = local_position;
        local_position_publisher.publish(local_position);
        LOG_MSG_STAMP("/dji_sdk/local_position", local_position, current_time, 1);
    }


    //update velocity msg
    if ((msg_flags & ENABLE_MSG_V)) {
        velocity.header.frame_id = "/world";
        velocity.header.stamp = current_time;
        velocity.ts = recv_sdk_std_msgs.time_stamp;
        velocity.vx = recv_sdk_std_msgs.v.x;
        velocity.vy = recv_sdk_std_msgs.v.y;
        velocity.vz = recv_sdk_std_msgs.v.z;
        velocity_publisher.publish(velocity);
        LOG_MSG_STAMP("/dji_sdk/velocity", velocity, current_time, 2);
    }

    //update acceleration msg
    if ((msg_flags & ENABLE_MSG_A)) {
        acceleration.header.frame_id = "/world";
        acceleration.header.stamp = current_time;
        acceleration.ts = recv_sdk_std_msgs.time_stamp;
        acceleration.ax = recv_sdk_std_msgs.a.x;
        acceleration.ay = recv_sdk_std_msgs.a.y;
        acceleration.az = recv_sdk_std_msgs.a.z;
        acceleration_publisher.publish(acceleration);
        LOG_MSG_STAMP("/dji_sdk/acceleration", acceleration, current_time, 2);
    }

    //update gimbal msg
    if ((msg_flags & ENABLE_MSG_GIMBAL)) {
        gimbal.header.frame_id = "/gimbal";
        gimbal.header.stamp= current_time;
        gimbal.ts = recv_sdk_std_msgs.time_stamp;
        gimbal.roll = recv_sdk_std_msgs.gimbal.x;
        gimbal.pitch = recv_sdk_std_msgs.gimbal.y;
        gimbal.yaw = recv_sdk_std_msgs.gimbal.z;
        gimbal_publisher.publish(gimbal);
        LOG_MSG_STAMP("/dji_sdk/gimbal", gimbal, current_time, 1);
    }

    //update odom msg
    if ((msg_flags & ENABLE_MSG_POS) && (msg_flags & ENABLE_MSG_Q) && (msg_flags & ENABLE_MSG_W) && (msg_flags & ENABLE_MSG_V)) {
        odometry.header.frame_id = "/world";
        odometry.header.stamp = current_time;
        odometry.pose.pose.position.x = local_position.x;
        odometry.pose.pose.position.y = local_position.y;
        odometry.pose.pose.position.z = local_position.z;
        odometry.pose.pose.orientation.w = attitude_quaternion.q0;
        odometry.pose.pose.orientation.x = attitude_quaternion.q1;
        odometry.pose.pose.orientation.y = attitude_quaternion.q2;
        odometry.pose.pose.orientation.z = attitude_quaternion.q3;
        odometry.twist.twist.angular.x = attitude_quaternion.wx;
        odometry.twist.twist.angular.y = attitude_quaternion.wy;
        odometry.twist.twist.angular.z = attitude_quaternion.wz;
        odometry.twist.twist.linear.x = velocity.vx;
        odometry.twist.twist.linear.y = velocity.vy;
        odometry.twist.twist.linear.z = velocity.vz;
        odometry_publisher.publish(odometry);
        LOG_MSG_STAMP("/dji_sdk/odometry", odometry, current_time, 1);
    }

    //update rc_channel msg
    if ((msg_flags & ENABLE_MSG_RC)) {
        rc_channels.header.frame_id = "/rc";
        rc_channels.header.stamp = current_time;
        rc_channels.ts = recv_sdk_std_msgs.time_stamp;
        rc_channels.pitch = recv_sdk_std_msgs.rc.pitch;
        rc_channels.roll = recv_sdk_std_msgs.rc.roll;
        rc_channels.mode = recv_sdk_std_msgs.rc.mode;
        rc_channels.gear = recv_sdk_std_msgs.rc.gear;
        rc_channels.throttle = recv_sdk_std_msgs.rc.throttle;
        rc_channels.yaw = recv_sdk_std_msgs.rc.yaw;
        rc_channels_publisher.publish(rc_channels);
        LOG_MSG_STAMP("/dji_sdk/rc_channels", rc_channels, current_time, 2);
    }

    //update compass msg
    if ((msg_flags & ENABLE_MSG_MAG)) {
        compass.header.frame_id = "/world";
        compass.header.stamp = current_time;
        compass.ts = recv_sdk_std_msgs.time_stamp;
        compass.x = recv_sdk_std_msgs.mag.x;
        compass.y = recv_sdk_std_msgs.mag.y;
        compass.z = recv_sdk_std_msgs.mag.z;
        compass_publisher.publish(compass);
        LOG_MSG_STAMP("/dji_sdk/compass", compass, current_time, 2);
    }


    //update flight_status 
    if ((msg_flags & ENABLE_MSG_STATUS)) {
        std_msgs::UInt8 msg;
        flight_status = recv_sdk_std_msgs.status;
        msg.data = flight_status;
        flight_status_publisher.publish(msg);
        LOG_MSG_STAMP("/dji_sdk/flight_status", msg, current_time, 1);
    }

    //update battery msg
    if ((msg_flags & ENABLE_MSG_BATTERY)) {
        power_status.percentage = recv_sdk_std_msgs.battery_remaining_capacity;
        power_status_publisher.publish(power_status);
        LOG_MSG_STAMP("/dji_sdk/power_status", power_status, current_time, 1);
    }

    //update flight control info
    if ((msg_flags & ENABLE_MSG_DEVICE)) {
        flight_control_info.cur_ctrl_dev_in_navi_mode = recv_sdk_std_msgs.ctrl_info.cur_ctrl_dev_in_navi_mode;
        flight_control_info.serial_req_status = recv_sdk_std_msgs.ctrl_info.serial_req_status;
        flight_control_info_publisher.publish(flight_control_info);
        LOG_MSG_STAMP("/dji_sdk/flight_control_info", flight_control_info, current_time, 2);
    }

    //update obtaincontrol msg
    if ((msg_flags & ENABLE_MSG_TIME)) {
        std_msgs::UInt8 msg;
        sdk_permission_opened = recv_sdk_std_msgs.obtained_control;
        msg.data = recv_sdk_std_msgs.obtained_control;
        sdk_permission_publisher.publish(msg);
        LOG_MSG_STAMP("/dji_sdk/sdk_permission", msg, current_time, 2);

        //update activation msg
        activated = recv_sdk_std_msgs.activation;
        msg.data = recv_sdk_std_msgs.activation;
        activation_publisher.publish(msg);
        LOG_MSG_STAMP("/dji_sdk/activation", msg, current_time, 2);
    }
}

int DJI_Setup(std::string serial_port, int baudrate) {
    int ret;
    char uart_name[32];
    strcpy(uart_name, serial_port.c_str());
    printf("Serial port: %s\n", uart_name);
    printf("Baudrate: %d\n", baudrate);
    printf("=========================\n");
    
    //Serial Port Init
    ret = Pro_Hw_Setup(uart_name, baudrate);
    if(ret < 0)
        return ret;

    //Setup Other Things
    DJI_Pro_Setup(NULL);
    return 0;
}
int DJISDKNode::init_parameters_and_activate(ros::NodeHandle& nh_private)
{
    std::string serial_name;
    int baud_rate;
    int app_id;
    int app_api_level;
    int app_version;
    std::string app_bundle_id;
    std::string enc_key;

    nh_private.param("serial_name", serial_name, std::string("/dev/cu.usbserial-A603T4HK"));
    nh_private.param("baud_rate", baud_rate, 230400);
    nh_private.param("app_id", app_id, 1022384);
    nh_private.param("app_api_level", app_api_level, 2);
    nh_private.param("app_version", app_version, 1);
    nh_private.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));
    nh_private.param("enc_key", enc_key,
            std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));
    nh_private.param("flyout_speed", flyout_speed, 8.0);
    nh_private.param("waypoint_speed", waypoint_speed, 4.0);
    nh_private.param("waypoint_region", waypoint_region, 5.0);
    nh_private.param("waypoint_turn_time", waypoint_turn_time, 1.5);
    int log_level {0};
    nh_private.param("telemetry_log_level", log_level, 0);
    std::string debug_file_name;
    //nh_private.param("debug_file_name", debug_file_name, std::string("/home/ubuntu/wp_test.log"));
    debug_file_name = BagLogger::instance()->getLogFileName("TRAJ");
    if ((debug_file = fopen(debug_file_name.c_str(), "a")) != NULL) {
        fprintf(debug_file, "\n\n################################\n\n");
    }

    if (log_level > 0)
    {
        // Start bag logger
        ROS_INFO("DJI: Telemetry logger ON level %d", log_level);
        BagLogger::instance()->startLogging("DJI", log_level);
    }
    else
    {
        ROS_INFO("DJI: Telemetry logger OFF");
    }

    // activation
    user_act_data.app_id = app_id;
    user_act_data.app_api_level = app_api_level;
    user_act_data.app_ver = SDK_VERSION;
    strcpy((char*) user_act_data.app_bundle_id, app_bundle_id.c_str());
    user_act_data.app_key = app_key;
    strcpy(user_act_data.app_key, enc_key.c_str());

    printf("=================================================\n");
    printf("app id: %d\n", user_act_data.app_id);
    printf("api level: %d\n", user_act_data.app_api_level);
    printf("app version: 0x0%X\n", user_act_data.app_ver);
    printf("app key: %s\n", user_act_data.app_key);
    printf("=================================================\n");
    ROS_INFO("DJI: speed: %f:%f, region: %f, turn_time: %f",
            flyout_speed, waypoint_speed, waypoint_region, waypoint_turn_time);

    if (DJI_Setup(serial_name.c_str(), baud_rate) < 0) {
        printf("Serial Port Cannot Open\n");
        return -1;
    }
    
    DJI_Pro_Activate_API(&user_act_data, NULL);
    DJI_Pro_Register_Broadcast_Callback(std::bind(&DJISDKNode::broadcast_callback, this));

    return 0;
}

DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
    init_publishers(nh);
    init_services(nh);
    init_actions(nh);
    init_parameters_and_activate(nh_private);
}

inline void DJISDKNode::gps_convert_ned(float &ned_x, float &ned_y,
            double gps_t_lon, double gps_t_lat,
            double gps_r_lon, double gps_r_lat)
{
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}

dji_sdk::LocalPosition DJISDKNode::gps_convert_ned(dji_sdk::GlobalPosition loc)
{
    dji_sdk::LocalPosition local;
    gps_convert_ned(local.x, local.y,
        loc.longitude, loc.latitude,
        global_position_ref.longitude, global_position_ref.latitude
    );
    local.z = loc.height;
    return local;
}

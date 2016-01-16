#include <dji_sdk/dji_sdk_node.h>
#include <algorithm>
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))

bool DJISDKNode::process_waypoint(dji_sdk::Waypoint new_waypoint) 
{
    double dst_latitude = new_waypoint.latitude;
    double dst_longitude = new_waypoint.longitude;
    float dst_altitude = new_waypoint.altitude;

    double org_latitude = global_position.latitude;
    double org_longitude = global_position.longitude;
    float org_altitude = local_position.z;

    double dis_x, dis_y;
    double diff_x, diff_y;
    float dis_z;

    dis_x = dst_latitude - org_latitude;
    dis_y = dst_longitude - org_longitude;
    dis_z = dst_altitude - org_altitude;

    ROS_INFO("WWW: Coord dst %f  %f  %f", dst_latitude, dst_longitude, dst_altitude);
    ROS_INFO("WWW: Coord org %f  %f  %f", org_latitude, org_longitude, org_altitude);
    ROS_INFO("WWW: Coord diff %f  %f  %f", dis_x, dis_y, dis_z);
    double det_x,det_y;
    float det_z;

    attitude_data_t user_ctrl_data;
    user_ctrl_data.ctrl_flag = 0x90;
    user_ctrl_data.thr_z = dst_altitude;
    user_ctrl_data.yaw = new_waypoint.heading;

    int latitude_progress = 0; 
    int longitude_progress = 0; 
    int altitude_progress = 0; 

    while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress <100) {
        if(waypoint_navigation_action_server->isPreemptRequested()) {
            return false;
        }

        double d_lon = dst_longitude - global_position.longitude;
        double d_lat = dst_latitude - global_position.latitude;
        diff_x = ((d_lat) *C_PI/180) * C_EARTH;
        diff_y = ((d_lon) * C_PI/180) * C_EARTH * cos((dst_latitude)*C_PI/180);
        user_ctrl_data.roll_or_x = diff_x;
        user_ctrl_data.pitch_or_y = diff_y;
        ROS_INFO("WWW: Setpoint %f  %f  %f", diff_x, diff_y, (double)user_ctrl_data.thr_z);
        DJI_Pro_Attitude_Control(&user_ctrl_data);

        det_x = (100 * (dst_latitude - global_position.latitude))/dis_x;
        det_y = (100 * (dst_longitude - global_position.longitude))/dis_y;
        det_z = (100 * (dst_altitude - local_position.z))/dis_z;
        ROS_INFO("WWW: Global Pos %f  %f  %f", (double)global_position.latitude, (double)global_position.longitude, (double)local_position.z);
        ROS_INFO("WWW: Det %f  %f  %f", det_x, det_y, (double)det_z);


        latitude_progress = 100 - std::abs((int) det_x);
        longitude_progress = 100 - std::abs((int) det_y);
        altitude_progress = 100 - std::abs((int) det_z);
        ROS_INFO("WWW: Progress before %d  %d  %d", latitude_progress, longitude_progress, altitude_progress);

        if (std::abs(diff_x) < 15) latitude_progress = 100;
        if (std::abs(diff_y) < 15) longitude_progress = 100;
        if (std::abs(dst_altitude - local_position.z) < 0.5) altitude_progress = 100;
        ROS_INFO("WWW: Progress after %d  %d  %d", latitude_progress, longitude_progress, altitude_progress);

        waypoint_navigation_feedback.latitude_progress = latitude_progress;
        waypoint_navigation_feedback.longitude_progress = longitude_progress;
        waypoint_navigation_feedback.altitude_progress = altitude_progress;
        waypoint_navigation_action_server->publishFeedback(waypoint_navigation_feedback);

        usleep(20000);

    }
    ros::Duration(new_waypoint.staytime).sleep();
    return true;
}

inline void DJISDKNode::vector_to_waypoint(Eigen::Vector3d& vector, WaypointData& wp)
{
    vector_between_locations(vector, global_position, wp.global_location);

    return;
}

inline void DJISDKNode::vector_between_locations(Eigen::Vector3d& vector,
        dji_sdk::GlobalPosition& from, dji_sdk::GlobalPosition& to)
{
    double d_lon = to.longitude - from.longitude;
    double d_lat = to.latitude - from.latitude;
    vector[0] = DEG2RAD(d_lat) * C_EARTH;
    vector[1] = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(to.latitude));
    vector[2] = to.altitude - from.altitude;

    return;
}

inline void DJISDKNode::vector_between_locations(Eigen::Vector3d& vector,
        dji_sdk::LocalPosition& from, dji_sdk::LocalPosition& to)
{
    vector[0] = to.x - from.x;
    vector[1] = to.y - from.y;
    vector[2] = to.z - from.z;

    return;
}

inline void DJISDKNode::global_to_local(Eigen::Vector3d& local, dji_sdk::GlobalPosition& global)
{
    double d_lon = global.longitude - global_position.longitude;
    double d_lat = global.latitude - global_position.latitude;
    double d_alt = global.altitude - global_position.altitude;
    local[0] = (DEG2RAD(d_lat) * C_EARTH) + local_position.x;
    local[1] = (DEG2RAD(d_lon) * C_EARTH *
            cos(DEG2RAD(global.latitude))) + local_position.y;
    local[2] = d_alt + local_position.z;

    return;
}

void DJISDKNode::send_velocity_setpoint(Eigen::Vector3d direction, double speed, int yaw)
{
    velocity_setpoint.ctrl_flag = 0x40;
    velocity_setpoint.roll_or_x = direction[0] * speed;
    velocity_setpoint.pitch_or_y = direction[1] * speed;
    velocity_setpoint.thr_z = direction[2] * speed;
    velocity_setpoint.yaw = yaw;

    DJI_Pro_Attitude_Control(&velocity_setpoint);
}
void DJISDKNode::log_waypoint(const WaypointData& wp) {
    debug_log("    Speed: %f    Region: %f    Heading: %d    Loiter: %d\n",
            wp.speed, wp.region, wp.heading, wp.loiter);
    debug_log("    Global: %f,  %f,  %f\n",
            wp.global_location.latitude, wp.global_location.longitude, wp.global_location.altitude);
    debug_log("    Local: %f,  %f,  %f\n",
            wp.local_location[0], wp.local_location[1], wp.local_location[2]);
    debug_log("    Vector: %f,  %f,  %f\n",
            wp.direction[0], wp.direction[1], wp.direction[2]);
}

void DJISDKNode::log_waypoints() {
    debug_log("Current postion\n");
    debug_log("    Global: %f, %f, %f\n", 
        global_position.latitude, global_position.longitude, global_position.altitude);
    debug_log("    Local: %f, %f, %f\n", 
        local_position.x, local_position.y, local_position.z);
    for (int i = 0; i < waypoints.size(); i++) {
        debug_log("WAYPOINT # %d\n", i);
        log_waypoint(waypoints[i]);
    }
}

bool DJISDKNode::init_waypoints(const dji_sdk::WaypointList& wp_list)
{
    waypoints.clear();
    ROS_INFO("WWW: Initializing %ld waypoints", wp_list.waypoint_list.size());
    for (int i = 0; i < wp_list.waypoint_list.size(); i++) {
        const dji_sdk::Waypoint wp = wp_list.waypoint_list[i];
        WaypointData wpd;
        wpd.index = i;
        wpd.global_location.latitude = wp.latitude;
        wpd.global_location.longitude = wp.longitude;
        wpd.global_location.altitude = global_position.altitude + wp.altitude;
        global_to_local(wpd.local_location, wpd.global_location);
        wpd.loiter = wp.staytime;
        wpd.heading = wp.heading;
        wpd.region = waypoint_region;
        wpd.speed = waypoint_speed;
        waypoints.push_back(wpd);
    }

    // Pre calculate all the direction vectors between waypoints
    for (int i = 0; i < waypoints.size()-1; i++) {
        vector_between_locations(waypoints[i].direction,
                waypoints[i].global_location , waypoints[i+1].global_location);
    }

    log_waypoints();

    return true;
}

double DJISDKNode::time_to_turn(double dist, double speed, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double theta = angle_beween_vectors(v1, v2);
    double radius = (dist * sin(theta / 2.0)) / sin((C_PI - theta) / 2.0);
    double sector_length = radius * (C_PI - theta);
    double t = abs(sector_length / speed);
    debug_log("    theta: %f,  radius: %f,  sector: %f,  time: %f\n",
            theta, radius, sector_length, t);
    return t;
}

double DJISDKNode::turn_duration(WaypointData& wp) {
//    Eigen::Vector3d cur_vec;
//    vector_to_waypoint(cur_vec, wp);
//    double dist = cur_vec.norm();
//    cur_vec.normalize();
//    double turn_time = time_to_turn(dist, waypoint_speed, cur_vec, wp.direction);
    // Initially just a fixed time turn duration
//    return turn_time;
    return 0.5;
}

bool DJISDKNode::fly_to_waypoint(WaypointData& wp) {

    waypoint_navigation_feedback.index_progress = wp.index;
    double x_dist = wp.local_location[0] - local_position.x;
    double y_dist = wp.local_location[1] - local_position.y;
    double z_dist = wp.local_location[2] - local_position.z;

    debug_log("FLYING TO WAYPOINT %d  %f, %f, %f\n", 
              wp.index, x_dist, y_dist, z_dist);
    debug_log("Current postion\n");
    debug_log("    Global: %f, %f, %f\n", 
        global_position.latitude, global_position.longitude, global_position.altitude);
    debug_log("    Local: %f, %f, %f\n", 
        local_position.x, local_position.y, local_position.z);
    for (;;) {
        if(waypoint_navigation_action_server->isPreemptRequested()) {
            return false;
        }
        Eigen::Vector3d cur_vec;
        vector_to_waypoint(cur_vec, wp);
        double dist = cur_vec.norm();
        debug_log("    DIST: %f  ", dist);
        if (dist < wp.region) {
            debug_log("    *** WITHIN RANGE ***\n", dist);
            return true;
        }

        cur_vec.normalize();
        debug_log(" X:%f  Y:%f  Z:%f  H:%d\n", cur_vec[0], cur_vec[1], cur_vec[2], wp.heading);
        send_velocity_setpoint(cur_vec, wp.speed, wp.heading);

        double det_x = wp.local_location[0] - local_position.x;
        double det_y = wp.local_location[1] - local_position.y;
        double det_z = wp.local_location[2] - local_position.z;
        int latitude_progress = 100 - std::abs((int) (det_x / x_dist));
        int longitude_progress = 100 - std::abs((int) (det_y / y_dist));
        int altitude_progress = 100 - std::abs((int) (det_z / z_dist));
        waypoint_navigation_feedback.latitude_progress = latitude_progress;
        waypoint_navigation_feedback.longitude_progress = longitude_progress;
        waypoint_navigation_feedback.altitude_progress = altitude_progress;
        waypoint_navigation_action_server->publishFeedback(waypoint_navigation_feedback);

        usleep(20000);
    }

    return true;
}

bool DJISDKNode::turn_at_waypoint(WaypointData& wp, WaypointData& wpn) {
    debug_log("===== TURNING AT WAYPOINT\n");
    double duration = turn_duration(wp);
    double delta_num = 50 / duration;
    Eigen::Vector3d cur_vec;
    debug_log("    FROM: %f  %f  %f\n", cur_vec[0], cur_vec[1], cur_vec[2]);
    debug_log("      TO: %f  %f  %f\n",
            wp.direction[0], wp.direction[1], wp.direction[2]);
    vector_to_waypoint(cur_vec, wp);
    cur_vec.normalize();
    Eigen::Vector3d delta_vec = (wp.direction - cur_vec) / delta_num;
    for (double end_time = ros::Time::now().toSec() + duration;
            ros::Time::now().toSec() < end_time;) {
        cur_vec = cur_vec + delta_vec;
        debug_log("     SET: %f  %f  %f\n", cur_vec[0], cur_vec[1], cur_vec[2]);
        send_velocity_setpoint(cur_vec, waypoint_speed, wp.heading);
        usleep(20000);
    }
    return true;
}

bool DJISDKNode::loiter_at_waypoint(WaypointData& wp) {
    return true;
}

bool DJISDKNode::fly_waypoints()
{
    for (int i = 0; i < waypoints.size(); i++) {
        if (!fly_to_waypoint(waypoints[i])) {
            return false;
        }

        if (i < (waypoints.size() - 1)) {
            if (waypoints[i].loiter > 0) {
                if (!loiter_at_waypoint(waypoints[i])) {
                    return false;
                }
            }
            else
            {
                if (!turn_at_waypoint(waypoints[i], waypoints[i+1])) {
                    return false;
                }
            }
        }
    }

    debug_log("\nFINISHED WITH WAYPOINT LIST\n");

    return true;
}

bool DJISDKNode::drone_task_action_callback(const dji_sdk::DroneTaskGoalConstPtr& goal)
{
  uint8_t request_action = goal->task;

  if (request_action == 1)
  {
     //takeoff
     DJI_Pro_Status_Ctrl(4, 0);
  }
  else if (request_action == 2)
  {
     //landing
     DJI_Pro_Status_Ctrl(6, 0);
  }
  else if (request_action == 3)
  {
     //gohome
     DJI_Pro_Status_Ctrl(1, 0);
  }

  drone_task_feedback.progress = 1;
  drone_task_action_server->publishFeedback(drone_task_feedback);
  drone_task_action_server->setSucceeded();
  
  return true;
}

bool DJISDKNode::local_position_navigation_action_callback(const dji_sdk::LocalPositionNavigationGoalConstPtr& goal)
{
  /*IMPORTANT*/
  /*
     There has been declared a pointer `local_navigation_action` as the function parameter,
     However, it is the `local_navigation_action_server` that we should use.
     If `local_navigation_action` is used instead, there will be a runtime sengmentation fault.

     so interesting
  */

  float dst_x = goal->x;
  float dst_y = goal->y;
  float dst_z = goal->z;

  float org_x = local_position.x;
  float org_y = local_position.y;
  float org_z = local_position.z;

  float dis_x = dst_x - org_x;
  float dis_y = dst_y - org_y;
  float dis_z = dst_z - org_z; 

  float det_x, det_y, det_z;

  attitude_data_t user_ctrl_data;
  user_ctrl_data.ctrl_flag = 0x90;
  user_ctrl_data.thr_z = dst_z;
  user_ctrl_data.yaw = 0;

  int x_progress = 0; 
  int y_progress = 0; 
  int z_progress = 0; 
  while (x_progress < 100 || y_progress < 100 || z_progress <100) {

     user_ctrl_data.roll_or_x = dst_x - local_position.x;
     user_ctrl_data.pitch_or_y = dst_y - local_position.y;

     DJI_Pro_Attitude_Control(&user_ctrl_data);

     det_x = (100 * (dst_x - local_position.x)) / dis_x;
     det_y = (100 * (dst_y - local_position.y)) / dis_y;
     det_z = (100 * (dst_z - local_position.z)) / dis_z;

     x_progress = 100 - (int)det_x;
     y_progress = 100 - (int)det_y;
     z_progress = 100 - (int)det_z;

     //lazy evaluation
     if (std::abs(dst_x - local_position.x) < 0.1) x_progress = 100;
     if (std::abs(dst_y - local_position.y) < 0.1) y_progress = 100;
     if (std::abs(dst_z - local_position.z) < 0.1) z_progress = 100;

     local_position_navigation_feedback.x_progress = x_progress;
     local_position_navigation_feedback.y_progress = y_progress;
     local_position_navigation_feedback.z_progress = z_progress;
     local_position_navigation_action_server->publishFeedback(local_position_navigation_feedback);

     usleep(20000);
  }

  local_position_navigation_result.result = true;
  local_position_navigation_action_server->setSucceeded(local_position_navigation_result);

  return true;
}

bool DJISDKNode::global_position_navigation_action_callback(const dji_sdk::GlobalPositionNavigationGoalConstPtr& goal)
{
    double dst_latitude = goal->latitude;
    double dst_longitude = goal->longitude;
    float dst_altitude = goal->altitude;

    double org_latitude = global_position.latitude;
    double org_longitude = global_position.longitude;
    float org_altitude = global_position.altitude;

    double dis_x, dis_y;
    float dis_z;

    dis_x = dst_latitude - org_latitude;
    dis_y = dst_longitude - org_longitude;
    dis_z = dst_altitude - org_altitude;

    double det_x, det_y;
    float det_z;

    attitude_data_t user_ctrl_data;
    user_ctrl_data.ctrl_flag = 0x90;
    user_ctrl_data.thr_z = dst_altitude;
    user_ctrl_data.yaw = 0;

    int latitude_progress = 0; 
    int longitude_progress = 0; 
    int altitude_progress = 0; 

    while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress < 100) {

		double d_lon = dst_longitude - global_position.longitude;
		double d_lat = dst_latitude - global_position.latitude;
		user_ctrl_data.roll_or_x = ((d_lat) *C_PI/180) * C_EARTH;
		user_ctrl_data.pitch_or_y = ((d_lon) * C_PI/180) * C_EARTH * cos((dst_latitude)*C_PI/180);

         DJI_Pro_Attitude_Control(&user_ctrl_data);

         det_x = (100* (dst_latitude - global_position.latitude))/dis_x;
         det_y = (100* (dst_longitude - global_position.longitude))/dis_y;
         det_z = (100* (dst_altitude - global_position.altitude))/dis_z;


         latitude_progress = 100 - (int)det_x;
         longitude_progress = 100 - (int)det_y;
         altitude_progress = 100 - (int)det_z;

         //lazy evaluation
         if (std::abs(dst_latitude - global_position.latitude) < 0.00001) latitude_progress = 100;
         if (std::abs(dst_longitude - global_position.longitude) < 0.00001) longitude_progress = 100;
         if (std::abs(dst_altitude - global_position.altitude) < 0.12) altitude_progress = 100;


         global_position_navigation_feedback.latitude_progress = latitude_progress;
         global_position_navigation_feedback.longitude_progress = longitude_progress;
         global_position_navigation_feedback.altitude_progress = altitude_progress;
         global_position_navigation_action_server->publishFeedback(global_position_navigation_feedback);

         usleep(20000);

    }

    global_position_navigation_result.result = true;
    global_position_navigation_action_server->setSucceeded(global_position_navigation_result);

    return true;
}

bool DJISDKNode::waypoint_navigation_action_callback(const dji_sdk::WaypointNavigationGoalConstPtr& goal)
{
//    dji_sdk::WaypointList new_waypoint_list;
//    new_waypoint_list = goal->waypoint_list;
//
//    bool isSucceeded;
//    ROS_INFO("WWW: Processing %ld waypoints", new_waypoint_list.waypoint_list.size());
//    for (int i = 0; i < new_waypoint_list.waypoint_list.size(); i++) {
//     const dji_sdk::Waypoint new_waypoint = new_waypoint_list.waypoint_list[i];
//     waypoint_navigation_feedback.index_progress = i;
//     isSucceeded = process_waypoint(new_waypoint);
//     if(!isSucceeded) {
//        waypoint_navigation_result.result = false;
//        waypoint_navigation_action_server->setPreempted(waypoint_navigation_result);
//        return false;
//     }
//    }

    if (!init_waypoints(goal->waypoint_list) || !fly_waypoints())
    {
        waypoint_navigation_result.result = false;
        waypoint_navigation_action_server->setPreempted(waypoint_navigation_result);
        return false;
    }

    waypoint_navigation_result.result = true;
    waypoint_navigation_action_server->setSucceeded(waypoint_navigation_result);

    return true;
}


/*
 * bag_logger.h
 *
 */

#ifndef BAG_LOGGER_H_
#define BAG_LOGGER_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>

#include <ros/ros.h>
#include <rosbag/bag.h>

#define DEFAULT_BAG_DIR "/home/ubuntu/testdata/"

#define LOG_MSG(topic, msg) { if (BagLogger::instance()->isLogging()) { \
            BagLogger::instance()->bag.write((topic), ros::Time::now(), (msg)); } }

class BagLogger {
    static BagLogger *s_instance_;

    bool is_logging_ {false};

    std::string file_name_;

    std::string log_name_prefix_ {""};

public:
    BagLogger() {};

    ~BagLogger() {};

    rosbag::Bag bag;

    static BagLogger *instance() {
        if (!s_instance_)
        {
            s_instance_ = new BagLogger;
        }
        return s_instance_;
    }

    bool isLogging() { return is_logging_; }

    std::string getLogFileName(std::string prefix) {
        log_name_prefix_ = prefix;
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        std::string format_str = DEFAULT_BAG_DIR+prefix+"-%Y-%m-%d-%H-%M-%S.bag";

        strftime(buffer, 80, format_str.c_str(), timeinfo);
//        strftime(buffer,80,"/home/ubuntu/testdata/%Y-%m-%d-%H-%M-%S.bag",timeinfo);
        std::string str(buffer);

        return str;
    }

    void startLogging(std::string prefix) {
        if (is_logging_)
        {
            ROS_INFO("Closing bag file %s", file_name_.c_str());
            bag.close();
        }
        file_name_ = getLogFileName(prefix);
        bag.open(file_name_, rosbag::bagmode::Write);
        ROS_INFO("Opening bag file %s", file_name_.c_str());
        is_logging_ = true;
    }

    void stopLogging() {
        if (is_logging_)
        {
            ROS_INFO("Closing bag file %s", file_name_.c_str());
            bag.close();
        }
        is_logging_ = false;
        return;
    };
};

#endif /* BAG_LOGGER_H_ */

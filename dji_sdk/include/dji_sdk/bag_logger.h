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

#define LOG_MSG(topic, msg, level) { LOG_MSG_STAMP(topic, msg, ros::Time::now(), level) }

#define LOG_MSG_STAMP(topic, msg, t, level) { if (BagLogger::instance()->isLogging() && \
                                                 (level) <= BagLogger::instance()->getLogLevel()) { \
    BagLogger::instance()->bag.write((topic), (t), (msg)); } }

class BagLogger {
    static BagLogger *s_instance_;

    unsigned log_level_ {0};

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

    std::string getLogFileName(std::string prefix) {
        log_name_prefix_ = prefix;
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        std::string format_str = DEFAULT_BAG_DIR+prefix+"-%Y-%m-%d-%H-%M-%S.bag";

        strftime(buffer, 80, format_str.c_str(), timeinfo);
        std::string str(buffer);

        return str;
    }

    void startLogging(std::string prefix, unsigned log_level) {
        if (is_logging_)
        {
            ROS_INFO("Closing bag file %s", file_name_.c_str());
            bag.close();
            log_level_ = 0;
            is_logging_ = false;
        }

        if (log_level > 0)
        {
            file_name_ = getLogFileName(prefix);
            bag.open(file_name_, rosbag::bagmode::Write);
            ROS_INFO("Opening bag file %s", file_name_.c_str());
            log_level_ = log_level;
            is_logging_ = true;
        }
    }

    void stopLogging() {
        if (is_logging_)
        {
            log_level_ = 0;
            is_logging_ = false;
            ROS_INFO("Closing bag file %s", file_name_.c_str());
            bag.close();
        }
        return;
    }
    unsigned getLogLevel() const {
        return log_level_;
    }

    bool isLogging() const {
        return is_logging_;
    }
};

#endif /* BAG_LOGGER_H_ */

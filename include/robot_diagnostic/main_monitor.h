/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the names of its contributors may be used to endorse 
 *     or promote products derived from this software without specific 
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Milan Michael
 *         
 *********************************************************************/

#ifndef MAIN_MONITOR_H
#define MAIN_MONITOR_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "topic_monitor.h"
#include "utility.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <robot_diagnostic/SystemStatus.h>
#include <robot_diagnostic/IndividualState.h>
#include <map>

/**
 * @class MainMointor
 * @brief This class will collect the topics from yaml file
 *        and pass those to ros diagnostics and mapping topic error to the given values.
*/
class MainMonitor
{
public:
    /**
     * @brief Constructor for MainMonitor
     * @brief nh ros node handler passed from main ros node
    */
    MainMonitor(ros::NodeHandle *nh);

    /**
     * @brief  To read all the data from yaml and add in the TopicInfo stuct
    */
    void GetTopicParams();

    ~MainMonitor()
    {
    }

private:
    ros::NodeHandle *nh_; /** variable to store the rosnode handler */

    /**
     * @brief A data structure defined for all storing the topics and related info
    */
    struct TopicInfo
    {
        std::vector<std::string> topic_name;
        std::vector<bool> is_header;
        std::vector<double> max_freq;
        std::vector<double> min_freq;
        std::vector<double> min_timestamp_diff;
        std::vector<double> max_timestamp_diff;
        std::vector<std::string> node_name;
        std::vector<std::string> warning_message;
        std::vector<std::string> error_message;
        std::vector<int> current_status;
        std::vector<std::string> current_message;
    };

    /**
     * @brief Subscriber callback funtion for the topic diagnostics.
     * @param msg is the diagnostics_msg/DiagnosticArray type having all the information reagarding the topics 
    */
    void DiagnosticsMessageCB(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

    /**
     * @brief Timer function for publishing the system state for much more understanding.
     * @param event is the time based event triggered by the ros createTimer.
    */
    void SystemState(const ros::TimerEvent &event);

    TopicInfo header_less_topic_infos; /** created a struct of TopicInfo for header less topics */
    TopicInfo header_topic_infos;      /** created a struct of TopicInfo for headered topics */
    TopicMonitor *topic_monitor;  /**  pointer of class TopicMonitor */

    ros::Subscriber diagnotics_sub; /**ROS Subscribers for diagnostic message. */

    ros::Publisher system_status_pub; /**ROS Publisher for system status message. */

    ros::Timer system_state_timer; /** ros timer for publishing system state. */

    std::map<std::string, int> status_map; /** used for the switch case for assigning topics to system status. */
};
#endif
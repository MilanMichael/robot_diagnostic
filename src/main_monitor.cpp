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

#include <robot_diagnostic/main_monitor.h>

MainMonitor::MainMonitor(ros::NodeHandle *nh)
{
    nh_ = nh;

    status_map.insert(std::pair<std::string, int>("GOOD", 0));
    status_map.insert(std::pair<std::string, int>("WARNING", 1));
    status_map.insert(std::pair<std::string, int>("ERROR", 2));

    GetTopicParams();

    topic_monitor = new TopicMonitor(*nh_);
    topic_monitor->AddTopic(*nh_, header_less_topic_infos.topic_name, header_less_topic_infos.min_freq, header_less_topic_infos.max_freq);
    topic_monitor->AddTopic(*nh_, header_topic_infos.topic_name, header_topic_infos.min_freq, header_topic_infos.max_freq, header_topic_infos.min_timestamp_diff, header_topic_infos.max_timestamp_diff);
    diagnotics_sub = nh->subscribe("/diagnostics", 1, &MainMonitor::DiagnosticsMessageCB, this);
    system_state_timer = nh->createTimer(ros::Duration(0.1), &MainMonitor::SystemState, this);
    system_status_pub=nh->advertise<robot_diagnostic::SystemStatus>("/system_status",1);

}

void MainMonitor::SystemState(const ros::TimerEvent &event)
{
    robot_diagnostic::SystemStatus state_status;
    robot_diagnostic::IndividualState indicidual_state;
    for (int i = 0; i <header_less_topic_infos.topic_name.size(); i++)
    {
        indicidual_state.node_name =  header_less_topic_infos.node_name[i];
        indicidual_state.topic_name =  header_less_topic_infos.topic_name[i];
        indicidual_state.health_status =  header_less_topic_infos.current_status[i] == 0 ? "GOOD" : (header_less_topic_infos.current_status[i] == 1 ? "WARNING" : "ERROR");
        indicidual_state.message = header_less_topic_infos.current_message[i];

        std::map<std::string, int>::iterator it = status_map.find(indicidual_state.health_status);
        switch ((*it).second)
        {
            case 0: 
                state_status.good_state.push_back(indicidual_state);
                break;
            case 1: 
                state_status.warning_state.push_back(indicidual_state);
                break;
            case 2: 
                state_status.error_state.push_back(indicidual_state);
                break;
            default :
                ROS_INFO("Unknown health status");
                break;
        }
    }
    for (int i = 0; i <header_topic_infos.topic_name.size(); i++)
    {
        indicidual_state.node_name =  header_topic_infos.node_name[i];
        indicidual_state.topic_name =  header_topic_infos.topic_name[i];
        indicidual_state.health_status = header_topic_infos.current_status[i] == 0 ? "GOOD" : (header_topic_infos.current_status[i] == 1 ? "WARNING" : "ERROR");
        indicidual_state.message = header_topic_infos.current_message[i];

        std::map<std::string, int>::iterator it = status_map.find(indicidual_state.health_status);
        switch ((*it).second)
        {
            case 0: 
                state_status.good_state.push_back(indicidual_state);
                break;
            case 1: 
                state_status.warning_state.push_back(indicidual_state);
                break;
            case 2: 
                state_status.error_state.push_back(indicidual_state);
                break;
            default :
                ROS_INFO("Unknown health status");
                break;
        }
    }
    state_status.system_level = state_status.error_state.size() > 0 ? 2 : (state_status.warning_state.size() > 0 ? 1 : 0);
    state_status.system_state = state_status.system_level == 0 ?  "System is Fine" : (state_status.system_level == 1 ? "System in Warning" : "System in Error");
    system_status_pub.publish(state_status);
}

void MainMonitor::DiagnosticsMessageCB(const diagnostic_msgs::DiagnosticArray::ConstPtr &msg)
{
    diagnostic_msgs::DiagnosticArray Message = *msg;
    std::vector<std::string>::iterator it;
    for (int i = 0; i < Message.status.size(); i++)
    {
        if (Message.status[i].level == diagnostic_msgs::DiagnosticStatus::OK)
        {
            std::string ok_topic_name = Utility::from_string_word(Message.status[i].name, 2);
            it = std::find(header_less_topic_infos.topic_name.begin(), header_less_topic_infos.topic_name.end(), ok_topic_name);
            if (it != header_less_topic_infos.topic_name.end())
            {
                if (header_less_topic_infos.current_status[it - header_less_topic_infos.topic_name.begin()] != 0)
                {
                    header_less_topic_infos.current_status[it - header_less_topic_infos.topic_name.begin()] = 0;
                    header_less_topic_infos.current_message[it - header_less_topic_infos.topic_name.begin()] = "ALL OK";
                }
            }
            it = std::find(header_topic_infos.topic_name.begin(), header_topic_infos.topic_name.end(), ok_topic_name);
            if (it != header_topic_infos.topic_name.end())
            {
                if (header_topic_infos.current_status[it - header_topic_infos.topic_name.begin()] != 0)
                {
                    header_topic_infos.current_status[it - header_topic_infos.topic_name.begin()] = 0;
                    header_topic_infos.current_message[it - header_topic_infos.topic_name.begin()] = "ALL OK";
                }
            }
        }
        else if (Message.status[i].level == diagnostic_msgs::DiagnosticStatus::WARN)
        {
            std::string warn_topic_name = Utility::from_string_word(Message.status[i].name, 2);
            it = std::find(header_less_topic_infos.topic_name.begin(), header_less_topic_infos.topic_name.end(), warn_topic_name);
            if (it != header_less_topic_infos.topic_name.end())
            {
                if (header_less_topic_infos.current_status[it - header_less_topic_infos.topic_name.begin()] != 1)
                {
                    header_less_topic_infos.current_status[it - header_less_topic_infos.topic_name.begin()] = 1;
                    header_less_topic_infos.current_message[it - header_less_topic_infos.topic_name.begin()] = header_less_topic_infos.warning_message[it - header_less_topic_infos.topic_name.begin()];
                }
            }
            it = std::find(header_topic_infos.topic_name.begin(), header_topic_infos.topic_name.end(), warn_topic_name);
            if (it != header_topic_infos.topic_name.end())
            {
                if (header_topic_infos.current_status[it - header_topic_infos.topic_name.begin()] != 1)
                {
                    header_topic_infos.current_status[it - header_topic_infos.topic_name.begin()] = 1;
                    header_topic_infos.current_message[it - header_topic_infos.topic_name.begin()] = header_topic_infos.warning_message[it - header_topic_infos.topic_name.begin()];
                }
            }
        }
        else if (Message.status[i].level == diagnostic_msgs::DiagnosticStatus::ERROR)
        {
            std::string error_topic_name = Utility::from_string_word(Message.status[i].name, 2);
            it = std::find(header_less_topic_infos.topic_name.begin(), header_less_topic_infos.topic_name.end(), error_topic_name);
            if (it != header_less_topic_infos.topic_name.end())
            {
                if (header_less_topic_infos.current_status[it - header_less_topic_infos.topic_name.begin()] != 2)
                {
                    header_less_topic_infos.current_status[it - header_less_topic_infos.topic_name.begin()] = 2;
                    header_less_topic_infos.current_message[it - header_less_topic_infos.topic_name.begin()] = header_less_topic_infos.error_message[it - header_less_topic_infos.topic_name.begin()];
                }
            }
            it = std::find(header_topic_infos.topic_name.begin(), header_topic_infos.topic_name.end(), error_topic_name);
            if (it != header_topic_infos.topic_name.end())
            {
                if (header_topic_infos.current_status[it - header_topic_infos.topic_name.begin()] != 2)
                {
                    header_topic_infos.current_status[it - header_topic_infos.topic_name.begin()] = 2;
                    header_topic_infos.current_message[it - header_topic_infos.topic_name.begin()] = header_topic_infos.error_message[it - header_topic_infos.topic_name.begin()];
                }
            }
        }
    }
}

void MainMonitor::GetTopicParams()
{
    bool end_reached = false;
    int topic_nos = 0;
    while (!end_reached)
    {
        char topic_name[100];
        sprintf(topic_name, "/Topic%d", topic_nos);
        if (nh_->hasParam(topic_name))
        {
            std::string name;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/name", name);
            bool isheader;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/isheader", isheader);
            double maxfreq;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/maxfreq", maxfreq);
            double minfreq;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/minfreq", minfreq);
            double mintimestamp;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/mintimestamp_diff", mintimestamp);
            double maxtimestamp;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/maxtimestamp_diff", maxtimestamp);
            std::string nodename;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/nodename", nodename);
            std::string warning_message;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/warning_message", warning_message);
            std::string error_message;
            nh_->getParam("/Topic" + Utility::to_string(topic_nos) + "/error_message", error_message);
            if (isheader)
            {
                header_topic_infos.topic_name.push_back(name);
                header_topic_infos.is_header.push_back(isheader);
                header_topic_infos.max_freq.push_back(maxfreq);
                header_topic_infos.min_freq.push_back(minfreq);
                header_topic_infos.min_timestamp_diff.push_back(mintimestamp);
                header_topic_infos.max_timestamp_diff.push_back(maxtimestamp);
                header_topic_infos.node_name.push_back(nodename);
                header_topic_infos.warning_message.push_back(warning_message);
                header_topic_infos.error_message.push_back(error_message);
                header_topic_infos.current_status.push_back(0);
                header_topic_infos.current_message.push_back("All Good");
            }
            else
            {
                header_less_topic_infos.topic_name.push_back(name);
                header_less_topic_infos.is_header.push_back(isheader);
                header_less_topic_infos.max_freq.push_back(maxfreq);
                header_less_topic_infos.min_freq.push_back(minfreq);
                header_less_topic_infos.min_timestamp_diff.push_back(0.0);
                header_less_topic_infos.max_timestamp_diff.push_back(0.0);
                header_less_topic_infos.node_name.push_back(nodename);
                header_less_topic_infos.warning_message.push_back(warning_message);
                header_less_topic_infos.error_message.push_back(error_message);
                header_less_topic_infos.current_status.push_back(0);
                header_less_topic_infos.current_message.push_back("All Good");
            }
        }
        else
        {
            end_reached = true;
        }
        topic_nos += 1;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_diagnostic");
    ros::NodeHandle nh("~");

    MainMonitor *main_monitor;
    main_monitor = new MainMonitor(&nh);

    double rate = 50;
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spin();
    }

    return 0;
}

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

    GetTopicParams();

    diagnotics_sub = nh->subscribe("/diagnostics", 1, &MainMonitor::DiagnosticsMessageCB, this);

    topic_monitor = new TopicMonitor(*nh_);
    topic_monitor->AddTopic(*nh_, all_topics.topic_name, all_topics.min_freq, all_topics.max_freq, all_topics.min_timestamp_diff,
                            all_topics.max_timestamp_diff, all_topics.is_header, all_topics.just_monitor);

    system_state_timer = nh->createTimer(ros::Duration(0.1), &MainMonitor::SystemState, this);
    system_status_pub = nh->advertise<robot_diagnostic::SystemStatus>("/system_status", 1);
}

void MainMonitor::SystemState(const ros::TimerEvent &event)
{
    robot_diagnostic::SystemStatus state_status;
    robot_diagnostic::IndividualState indicidual_state;
    for (int i = 0; i < all_topics.topic_name.size(); i++)
    {
        indicidual_state.node_name = all_topics.node_name[i];
        indicidual_state.topic_name = all_topics.topic_name[i];
        indicidual_state.health_status = all_topics.current_status[i] == diagnostic_msgs::DiagnosticStatus::OK ? "GOOD" : 
             (all_topics.current_status[i] == diagnostic_msgs::DiagnosticStatus::WARN ?
             "WARNING" : (all_topics.current_status[i] == diagnostic_msgs::DiagnosticStatus::ERROR ? "ERROR" : "Initialized"));
        indicidual_state.message = all_topics.current_message[i];

        switch (all_topics.current_status[i])
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
        default:
            state_status.error_state.push_back(indicidual_state);
            break;
        }
    }
    state_status.system_level = state_status.error_state.size() > 0 ? diagnostic_msgs::DiagnosticStatus::ERROR : 
        (state_status.warning_state.size() > 0 ? diagnostic_msgs::DiagnosticStatus::WARN : diagnostic_msgs::DiagnosticStatus::OK);
    state_status.system_state = state_status.system_level == 0 ? "System is Fine" : (state_status.system_level == 1 ? "System in Warning" : "System in Error");
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
            std::string ok_topic_name = Utility::WordFromString(Message.status[i].name, 2);
            it = std::find(all_topics.topic_name.begin(), all_topics.topic_name.end(), ok_topic_name);
            if (it != all_topics.topic_name.end())
            {
                if (all_topics.current_status[it - all_topics.topic_name.begin()] != diagnostic_msgs::DiagnosticStatus::OK)
                {
                    all_topics.current_status[it - all_topics.topic_name.begin()] = diagnostic_msgs::DiagnosticStatus::OK;
                    all_topics.current_message[it - all_topics.topic_name.begin()] = "ALL OK";
                }
            }
        }
        else if (Message.status[i].level == diagnostic_msgs::DiagnosticStatus::WARN)
        {
            std::string warn_topic_name = Utility::WordFromString(Message.status[i].name, 2);
            it = std::find(all_topics.topic_name.begin(), all_topics.topic_name.end(), warn_topic_name);
            if (it != all_topics.topic_name.end())
            {
                if (all_topics.current_status[it - all_topics.topic_name.begin()] != diagnostic_msgs::DiagnosticStatus::WARN)
                {
                    all_topics.current_status[it - all_topics.topic_name.begin()] = diagnostic_msgs::DiagnosticStatus::WARN;
                    all_topics.current_message[it - all_topics.topic_name.begin()] = all_topics.warning_message[it - all_topics.topic_name.begin()];
                }
            }
        }
        else if (Message.status[i].level == diagnostic_msgs::DiagnosticStatus::ERROR)
        {
            std::string error_topic_name = Utility::WordFromString(Message.status[i].name, 2);
            it = std::find(all_topics.topic_name.begin(), all_topics.topic_name.end(), error_topic_name);
            if (it != all_topics.topic_name.end())
            {
                if (all_topics.current_status[it - all_topics.topic_name.begin()] != diagnostic_msgs::DiagnosticStatus::ERROR)
                {
                    all_topics.current_status[it - all_topics.topic_name.begin()] = diagnostic_msgs::DiagnosticStatus::ERROR;
                    all_topics.current_message[it - all_topics.topic_name.begin()] = all_topics.error_message[it - all_topics.topic_name.begin()];
                }
            }
        }
    }
}

void MainMonitor::GetTopicParams()
{    
    XmlRpc::XmlRpcValue topicList;
    if (nh_->getParam("/Diagnostic_topics", topicList))
    {
        std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
        for (i = topicList.begin(); i != topicList.end(); i++)
        {
            all_topics.topic_name.push_back(i->second["name"]);
            all_topics.is_header.push_back(i->second["isheader"]);
            all_topics.max_freq.push_back(i->second["maxfreq"]);
            all_topics.min_freq.push_back(i->second["minfreq"]);
            all_topics.min_timestamp_diff.push_back(i->second["mintimestamp_diff"]);
            all_topics.max_timestamp_diff.push_back(i->second["maxtimestamp_diff"]);
            all_topics.node_name.push_back(i->second["nodename"]);
            all_topics.warning_message.push_back(i->second["warning_message"]);
            all_topics.error_message.push_back(i->second["error_message"]);
            all_topics.just_monitor.push_back(i->second["just_monitor"]);
            all_topics.current_status.push_back(3);
            all_topics.current_message.push_back("Topic Initialized , No new message from diagnostics");
        }
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

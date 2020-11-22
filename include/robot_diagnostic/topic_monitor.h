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

#ifndef TOPIC_MONITOR_H
#define TOPIC_MONITOR_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <boost/bind.hpp>


/**
 * @brief A data structure defined for all rostopic callbacks
 */

struct DummySubscriberData
{
public:
    static std::string md5;
    static std::string data_type;
    static std::string const &__s_getMD5Sum() { return md5; }
    static std::string const &__s_getDataType() { return data_type; }
    void deserialize(void *) {}
};
std::string DummySubscriberData::md5 = "*";
std::string DummySubscriberData::data_type = "/";

using namespace std;

/**
 * @class TopicMonitor.
 * @brief This class will act as a bridge for connecting the given topics to ros diagnostic package.
*/
class TopicMonitor
{
public:
    /**
     * @brief Constructor for TopicMonitor
     * @param nh is the ros node handler passed from main ros node. 
    */
    TopicMonitor(ros::NodeHandle &nh)
    {
        ROS_INFO("Initializing topic monitor...");
        updater = new diagnostic_updater::Updater(); // The diagnostic_updater object to provide diagnostics functionalities. 
        updater->setHardwareID("Main_Diagnotics");   // This ID will help you to identify the specific device, and here defined as none. 
        updater->force_update();
        ROS_INFO("Initialized ... Topic Monitor is up and running");
    }

    /**
     * @brief  TopicMonitor without header.
     * @param nh ros node handler passed from the main ros node.
     * @param topic_name  list of topics need to be monitored.
     * @param topic_min_freq mainimum required frequency of the topic.
     * @param topic_max_freq maximum required frequency of the topic.
    */
    void AddTopic(ros::NodeHandle &nh, std::vector<string> topic_names, std::vector<double> topic_min_freq, std::vector<double> topic_max_freq)
    {
        for (int i = header_less_topics.size(), j = header_less_topics.size(); i < topic_names.size() + j; i++)
        {
            header_less_topics.push_back(new diagnostic_updater::HeaderlessTopicDiagnostic(topic_names[i], *updater, 
                diagnostic_updater::FrequencyStatusParam(&topic_min_freq[i], &topic_max_freq[i], 0.1, 1)));
            ros::Subscriber *common_sub = new ros::Subscriber;
            *common_sub = nh.subscribe<DummySubscriberData>(topic_names[i], 1, 
                boost::bind(&TopicMonitor::CommonSubscriberCallback_, this, _1, header_less_topics[i]));
        }
        ros::Duration(1).sleep();
        updater->update();
    }

    /**
     * @brief  TopicMonitor with header.
     * @param nh ros node handler passed from the main ros node.
     * @param topic_name  list of topics need to be monitored.
     * @param topic_min_freq mainimum required frequency of the topic.
     * @param topic_max_freq maximum required frequency of the topic.
     * @param topic_min_timestampdiff minimum timestamp difference.
     * @param topic_max_timestampdiff maximum timestamp difference.
     * 
    */
    void AddTopic(ros::NodeHandle &nh, std::vector<string> topic_names, std::vector<double> topic_min_freq, 
        std::vector<double> topic_max_freq, std::vector<double> topic_min_timestampdiff, std::vector<double> topic_max_timestampdiff)
    {
        for (int i = header_topics.size(), j = header_topics.size(); i < topic_names.size() + j; i++)
        {
            header_topics.push_back(new diagnostic_updater::TopicDiagnostic(topic_names[i], *updater, 
                diagnostic_updater::FrequencyStatusParam(&topic_min_freq[i], &topic_max_freq[i], 0.1, 1), 
                diagnostic_updater::TimeStampStatusParam(topic_min_timestampdiff[i], topic_max_timestampdiff[i])));
            ros::Subscriber *common_sub = new ros::Subscriber;
            *common_sub = nh.subscribe<DummySubscriberData>(topic_names[i], 1, 
                boost::bind(&TopicMonitor::CommonSubscriberCallback, this, _1, header_topics[i]));
        }
        ros::Duration(1).sleep();
        updater->update();
    }

    /**
     * @brief  TopicMonitor with header.
     * @param nh ros node handler passed from the main ros node.
     * @param topic_name  list of topics need to be monitored.
     * @param topic_min_freq mainimum required frequency of the topic.
     * @param topic_max_freq maximum required frequency of the topic.
     * @param topic_min_timestampdiff minimum timestamp difference.
     * @param topic_max_timestampdiff maximum timestamp difference.
     * @param is_header is a vector of bool, for identifying topics with header and header less (true for header).
     * @param just_monitor is a vector of bool stating whether that topic need to be added to diagnostic updater (false if need to be added);
    */
    void AddTopic(ros::NodeHandle &nh, std::vector<string> topic_names, std::vector<double> topic_min_freq, 
        std::vector<double> topic_max_freq, std::vector<double> topic_min_timestampdiff, 
        std::vector<double> topic_max_timestampdiff, std::vector<bool> is_header, std::vector<bool> just_monitor)
    {
        for (int i = 0; i < topic_names.size(); i++)
        {
            if (is_header[i] && !just_monitor[i])
            {
                header_topics.push_back(new diagnostic_updater::TopicDiagnostic(topic_names[i], *updater, 
                diagnostic_updater::FrequencyStatusParam(&topic_min_freq[i], &topic_max_freq[i], 0.1, 1), 
                diagnostic_updater::TimeStampStatusParam(topic_min_timestampdiff[i], topic_max_timestampdiff[i])));
                ros::Subscriber *common_sub = new ros::Subscriber;
                *common_sub = nh.subscribe<DummySubscriberData>(topic_names[i], 1, 
                    boost::bind(&TopicMonitor::CommonSubscriberCallback, this, _1, header_topics.back()));
            }
            else if (!just_monitor[i])
            {
                header_less_topics.push_back(new diagnostic_updater::HeaderlessTopicDiagnostic(topic_names[i], *updater, 
                diagnostic_updater::FrequencyStatusParam(&topic_min_freq[i], &topic_max_freq[i], 0.1, 1)));
                ros::Subscriber *common_sub = new ros::Subscriber;
                *common_sub = nh.subscribe<DummySubscriberData>(topic_names[i], 1, 
                    boost::bind(&TopicMonitor::CommonSubscriberCallback_, this, _1, header_less_topics.back()));
            }
        }
        ros::Duration(1).sleep();
        updater->update();
    }

    /**
     * @brief  Destructor for the TopicMonitor.
    */
    ~TopicMonitor()
    {
        delete updater;
        delete header_less_topics[header_less_topics.size()];
        delete header_topics[header_topics.size()];
    }

private:

    /**
     * @brief Common subscriber callback funtion for all header less topics.
     * @param data is the dymmmy Msg type created for the subscribing topic.
     * @param topic_name_diagnostic passed diagnotic instance of the topic.
    */
    void CommonSubscriberCallback_(const boost::shared_ptr<DummySubscriberData const> &data, 
        diagnostic_updater::HeaderlessTopicDiagnostic *topic_name_diagnostic_)
    {
        topic_name_diagnostic_->tick();
        updater->update();
    }

    /**
     * @brief Common subscriber callback funtion for all topics with header.
     * @param data is the dymmmy Msg type created for the subscribing topic.
     * @param topic_name_diagnostic passed diagnotic instance of the topic.
    */
    void CommonSubscriberCallback(const boost::shared_ptr<DummySubscriberData const> &data, 
        diagnostic_updater::TopicDiagnostic *topic_name_diagnostic_)
    {
        topic_name_diagnostic_->tick(ros::Time::now());
        updater->update();
    }

    std::vector<diagnostic_updater::TopicDiagnostic *> header_topics; // list of header topics need to monitor. 
    std::vector<diagnostic_updater::HeaderlessTopicDiagnostic *> header_less_topics; // list of header less topics need to monitor.

    diagnostic_updater::Updater *updater; // Created an instance of the ros diagnostics.

};
#endif
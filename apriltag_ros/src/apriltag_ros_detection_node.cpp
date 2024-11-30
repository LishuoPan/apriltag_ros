//
// Created by lishuo on 11/16/24.
//


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <nlohmann/json.hpp>
#include <signal.h>
#include <fstream>


sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed
class DetectionNode
{
public:

    using json = nlohmann::json;

    DetectionNode() : nh_priv_("~")
    {
        // ---------------- ROS params -----------------
        this->nh_priv_.getParam("ROBOT_ID", ROBOT_ID);
        this->nh_priv_.getParam("NUM_TARGETS", NUM_TARGETS);
        this->nh_priv_.getParam("rate", rate);
        this->nh_priv_.getParam("CONFIG_FILENAME", CONFIG_FILENAME);

        for (size_t i = 0; i < NUM_TARGETS+1; ++i) {
            if (i == ROBOT_ID) {
                continue;
            }
            target_ids_.push_back(i);
        }
        assert(target_ids_.size() == NUM_TARGETS);

        // ---------------- load json -----------------
        std::string experiment_config_filename = CONFIG_FILENAME;
        std::fstream experiment_config_fc(experiment_config_filename.c_str(), std::ios_base::in);
        json experiment_config_json = json::parse(experiment_config_fc);
        std::cout << "successfully load the json..." << "\n";

        // assign tags to each neighbor
        for (auto target_id:target_ids_) {
            std::vector<size_t> neighbor_tags = {experiment_config_json["vision"]["tags"][target_id][0], experiment_config_json["vision"]["tags"][target_id][1]};
            target_tags_.push_back(neighbor_tags);
        }
        assert(target_tags_.size() == target_ids_.size());

        // ---------- Subs and Pubs -------------------
        target_pubs_.resize(NUM_TARGETS);
        for (size_t i = 0; i < NUM_TARGETS; ++i) {
            size_t target_id = target_ids_[i];
            target_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>("uav"+std::to_string(ROBOT_ID)+"/target"+std::to_string(target_id), 10);
        }
        detection_timer_ = nh_.createTimer(ros::Duration(1/rate), std::bind(&DetectionNode::detection_timer_callback, this));
        std::cout << "initialized all subs&pubs\n";
    }

    ~DetectionNode()
    {
        std::cout << "DetectionNode destroyer called\n";
    }

    void stop();
    void detection_timer_callback();

private:
    int ROBOT_ID = 0;
    int NUM_TARGETS = 1;
    double rate = 60.0;
    std::string CONFIG_FILENAME;

    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    tf::TransformListener TF_listener_;
    std::vector<ros::Publisher> target_pubs_;
    ros::Timer detection_timer_;

    std::vector<std::vector<size_t>> target_tags_;
    std::vector<size_t> target_ids_;
};

void DetectionNode::stop()
{
    ros::Duration(0.1).sleep();
    ros::shutdown();
}

void DetectionNode::detection_timer_callback() {
    for (size_t i = 0; i < NUM_TARGETS; ++i) {
        size_t target_id = target_ids_.at(i);
        std::vector<size_t> tags = target_tags_.at(i);


        std::vector<tf::StampedTransform> transformStampedes;
        transformStampedes.resize(tags.size());
        std::cout << "start read transformStampedes\n";
        for (size_t j = 0; j < tags.size(); ++j) {
            try {
                TF_listener_.lookupTransform("/camera_link", "/tag_" + std::to_string(tags.at(j)),
                                             ros::Time(0), transformStampedes.at(j));
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }

        // get the lastest transformStamp
        size_t lastest_transform_index = 0;
        ros::Time lastest_timestamp = transformStampedes.at(0).stamp_;
        for (size_t j = 1; j < tags.size(); ++j) {
            ros::Time timestamp = transformStampedes.at(j).stamp_;
            if (timestamp > lastest_timestamp) {
                lastest_transform_index = j;
                lastest_timestamp = timestamp;
            }
        }
        tf::StampedTransform lastest_transformStamped = transformStampedes.at(lastest_transform_index);

        // Create a PoseStamped message
        geometry_msgs::PoseStamped pose_stamped_msg;
        // Set the header
        pose_stamped_msg.header.stamp = lastest_timestamp;
        pose_stamped_msg.header.frame_id = "uav"+std::to_string(ROBOT_ID)+"/target"+std::to_string(target_id);

        // Set position (from translation of transform)
        pose_stamped_msg.pose.position.x = lastest_transformStamped.getOrigin().x();
        pose_stamped_msg.pose.position.y = lastest_transformStamped.getOrigin().y();
        pose_stamped_msg.pose.position.z = lastest_transformStamped.getOrigin().z();
        // Set orientation (from rotation of transform)
        // Extract rotation (orientation)
        tf::Quaternion quat = lastest_transformStamped.getRotation();
        pose_stamped_msg.pose.orientation.x = quat.x();
        pose_stamped_msg.pose.orientation.y = quat.y();
        pose_stamped_msg.pose.orientation.z = quat.z();
        pose_stamped_msg.pose.orientation.w = quat.w();

        // publish the poseStamp
        target_pubs_[i].publish(pose_stamped_msg);

    }
}

/*******************************************************************************
* Main function
*******************************************************************************/
//alternatively to a global variable to have access to the method you can make STATIC the class method interested,
//but some class function may not be accessed: "this->" method cannot be used

void nodeobj_wrapper_function(int){
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "detection_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    // detection_node
    auto detection_node = std::make_shared<DetectionNode>();

    while (!node_shutdown_request){
        ros::spinOnce();
    }
    detection_node->stop();

    //ros::spin();
    //do pre-shutdown tasks
    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}
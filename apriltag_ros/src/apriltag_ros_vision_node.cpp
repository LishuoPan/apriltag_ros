#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <XmlRpcValue.h>

struct Tag {
    int id;
    double size;
    std::string name;  // Add name field, optional

    // Constructor for easy initialization
    Tag(int id_, double size_, std::string name_) : id(id_), size(size_), name(name_) {}
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "apriltag_vision_node");
    ros::NodeHandle nh;

    // Pub/sub
    // Container for storing the retrieved tags from the parameter server
    std::vector<Tag> standalone_tags;

    // Temporary variable to store the raw parameter data
    XmlRpc::XmlRpcValue raw_tags;

    // Retrieve the parameter as XmlRpcValue
    if (nh.getParam("/apriltag_ros_vision_node/standalone_tags", raw_tags)) {
        // Ensure the parameter is of the correct type (array)
        if (raw_tags.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < raw_tags.size(); ++i) {
                // Ensure each element in the array is a struct (map)
                if (raw_tags[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    // Retrieve id and size from the struct
                    int id = static_cast<int>(raw_tags[i]["id"]);
                    double size = static_cast<double>(raw_tags[i]["size"]);
                    
                    // Check if the name exists, otherwise provide a default name
                    std::string name = "Unnamed Tag";  // Default value
                    if (raw_tags[i].hasMember("name")) {
                        name = static_cast<std::string>(raw_tags[i]["name"]);
                    }

                    // Create a Tag object and add it to the standalone_tags vector
                    standalone_tags.emplace_back(id, size, name);
                }
            }

            // Print the retrieved tags
            for (const auto& tag : standalone_tags) {
                std::cout << "Tag ID: " << tag.id << ", Size: " << tag.size << ", Name: " << tag.name << std::endl;
            }
        } else {
            ROS_ERROR("standalone_tags parameter is not an array.");
        }
    } else {
        ROS_ERROR("Failed to retrieve 'standalone_tags' parameter.");
    }

    std::vector<ros::Publisher> pose_pubs(standalone_tags.size());
    for (int i = 0; i < standalone_tags.size(); ++i) {
        pose_pubs[i] = nh.advertise<geometry_msgs::Pose>("tag_"+std::to_string(standalone_tags.at(i).id)+"/pose", 10);
    }

    tf::TransformListener listener;

    ros::Rate loop_rate(60); // Publish at 60 Hz

    while (ros::ok()) {
        for (int i = 0; i < standalone_tags.size(); ++i) {
            tf::StampedTransform transformStamped;
            try{
                listener.lookupTransform("/camera_link", "/tag_"+std::to_string(standalone_tags.at(i).id),   
                                        ros::Time(0), transformStamped);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }

            // Create a Pose message
            geometry_msgs::Pose pose_msg;
            // Set position (from translation of transform)
            pose_msg.position.x = transformStamped.getOrigin().x();
            pose_msg.position.y = transformStamped.getOrigin().y();
            pose_msg.position.z = transformStamped.getOrigin().z();
            // Set orientation (from rotation of transform)
            // Extract rotation (orientation)
            tf::Quaternion quat = transformStamped.getRotation();
            pose_msg.orientation.x = quat.x();
            pose_msg.orientation.y = quat.y();
            pose_msg.orientation.z = quat.z();
            pose_msg.orientation.w = quat.w();

            // Publish the pose message
            pose_pubs[i].publish(pose_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
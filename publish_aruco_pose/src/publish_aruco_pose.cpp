#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Structure to hold the pose values
struct ArucoPose {
  double tx;
  double ty;
  double tz;
  double roll;
  double pitch;
  double yaw;
};

// Function to read the pose from a file
ArucoPose readPoseFromFile(const std::string& filename) {
  ArucoPose pose;
  std::ifstream file(filename);
  if(file.is_open()) {
    file >> pose.tx >> pose.ty >> pose.tz >> pose.roll >> pose.pitch >> pose.yaw;
    file.close();
    std::cout << "Read pose: "
              << pose.tx << " " << pose.ty << " " << pose.tz << " "
              << pose.roll << " " << pose.pitch << " " << pose.yaw << std::endl;
  } else {
    std::cerr << "Failed to open file: " << filename << std::endl;
  }
  return pose;
}

class ArucoStaticTFPublisher : public rclcpp::Node {
public:
  ArucoStaticTFPublisher(const ArucoPose &pose)
  : Node("aruco_static_tf_broadcaster")
  {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Create the TransformStamped message
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "map";         // Parent frame (adjust as needed)
    static_transform.child_frame_id = "camera_init";    // Child frame (adjust as needed)

    // Set the translation using the pose from the file
    static_transform.transform.translation.x = pose.tx;
    static_transform.transform.translation.y = pose.ty;
    static_transform.transform.translation.z = pose.tz;

    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(pose.roll, pose.pitch, pose.yaw);
    static_transform.transform.rotation.x = q.x();
    static_transform.transform.rotation.y = q.y();
    static_transform.transform.rotation.z = q.z();
    static_transform.transform.rotation.w = q.w();

    // Publish the static transform
    static_broadcaster_->sendTransform(static_transform);
    RCLCPP_INFO(this->get_logger(),
                "Published static transform: translation(%.2f, %.2f, %.2f) rotation(%.2f, %.2f, %.2f)",
                pose.tx, pose.ty, pose.tz, pose.roll, pose.pitch, pose.yaw);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Read the Aruco pose from file
  std::string filename = "robot_aruco_pose.txt";
  ArucoPose pose = readPoseFromFile(filename);

  // Create and spin the node that publishes the static transform
  auto node = std::make_shared<ArucoStaticTFPublisher>(pose);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

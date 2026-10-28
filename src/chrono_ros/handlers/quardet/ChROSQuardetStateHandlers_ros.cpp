// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Subprocess (chrono_ros_node) ROS publishing for quardet_sim state handlers.
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/quardet/ChROSQuardetIPC.h"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

// -----------------------------------------------------------------------------
// Wrench
// -----------------------------------------------------------------------------

void PublishQrdWrenchToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size != sizeof(ipc::QrdWrenchData)) return;
    ipc::QrdWrenchData header;
    std::memcpy(&header, data, sizeof(header));

    static std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> publishers;
    std::string topic(header.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<geometry_msgs::msg::WrenchStamped>(topic, 10);
    }

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp.sec = header.sec;
    msg.header.stamp.nanosec = header.nsec;
    msg.header.frame_id = std::string(header.frame_id);
    msg.wrench.force.x = header.force_x;
    msg.wrench.force.y = header.force_y;
    msg.wrench.force.z = header.force_z;
    msg.wrench.torque.x = header.torque_x;
    msg.wrench.torque.y = header.torque_y;
    msg.wrench.torque.z = header.torque_z;
    publishers[topic]->publish(msg);
}

// -----------------------------------------------------------------------------
// JointState (publisher)
// -----------------------------------------------------------------------------

void PublishQrdJointStateToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size != sizeof(ipc::QrdJointStateData)) return;
    ipc::QrdJointStateData header;
    std::memcpy(&header, data, sizeof(header));

    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> publishers;
    std::string topic(header.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<sensor_msgs::msg::JointState>(topic, 10);
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp.sec = header.sec;
    msg.header.stamp.nanosec = header.nsec;
    msg.header.frame_id = std::string(header.frame_id);
    for (uint32_t i = 0; i < header.num_joints; ++i) {
        msg.name.push_back(std::string(header.names[i]));
        msg.position.push_back(header.position[i]);
        msg.velocity.push_back(header.velocity[i]);
        msg.effort.push_back(header.effort[i]);
    }
    publishers[topic]->publish(msg);
}

// -----------------------------------------------------------------------------
// Odometry
// -----------------------------------------------------------------------------

void PublishQrdOdomToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size != sizeof(ipc::QrdOdomData)) return;
    ipc::QrdOdomData header;
    std::memcpy(&header, data, sizeof(header));

    static std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> publishers;
    std::string topic(header.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<nav_msgs::msg::Odometry>(topic, 10);
    }

    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = header.sec;
    msg.header.stamp.nanosec = header.nsec;
    msg.header.frame_id = std::string(header.frame_id);
    msg.child_frame_id = std::string(header.child_frame_id);
    msg.twist.twist.linear.x = header.lin_vel_x;
    msg.twist.twist.linear.y = header.lin_vel_y;
    msg.twist.twist.linear.z = header.lin_vel_z;
    msg.twist.twist.angular.x = header.ang_vel_x;
    msg.twist.twist.angular.y = header.ang_vel_y;
    msg.twist.twist.angular.z = header.ang_vel_z;
    msg.pose.pose.position.x = header.pos_x;
    msg.pose.pose.position.y = header.pos_y;
    msg.pose.pose.position.z = header.pos_z;
    msg.pose.pose.orientation.x = header.ori_x;
    msg.pose.pose.orientation.y = header.ori_y;
    msg.pose.pose.orientation.z = header.ori_z;
    msg.pose.pose.orientation.w = header.ori_w;
    publishers[topic]->publish(msg);
}

// -----------------------------------------------------------------------------
// TF (shared by ChROSQuardetTFHandler and ChROSQuardetCameraLinkTFHandler)
// -----------------------------------------------------------------------------

void PublishQrdTFToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size < sizeof(ipc::QrdTFHeader)) return;
    ipc::QrdTFHeader header;
    std::memcpy(&header, data, sizeof(header));

    size_t expected = sizeof(ipc::QrdTFHeader) + header.num_transforms * sizeof(ipc::QrdTFTransform);
    if (data_size != expected) return;

    static rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher;
    if (!publisher) {
        publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    }

    const auto* transforms = reinterpret_cast<const ipc::QrdTFTransform*>(data + sizeof(ipc::QrdTFHeader));

    tf2_msgs::msg::TFMessage msg;
    msg.transforms.resize(header.num_transforms);
    for (uint32_t i = 0; i < header.num_transforms; ++i) {
        msg.transforms[i].header.stamp.sec = header.sec;
        msg.transforms[i].header.stamp.nanosec = header.nsec;
        msg.transforms[i].header.frame_id = std::string(transforms[i].parent_frame);
        msg.transforms[i].child_frame_id = std::string(transforms[i].child_frame);
        msg.transforms[i].transform.translation.x = transforms[i].tx;
        msg.transforms[i].transform.translation.y = transforms[i].ty;
        msg.transforms[i].transform.translation.z = transforms[i].tz;
        msg.transforms[i].transform.rotation.x = transforms[i].rx;
        msg.transforms[i].transform.rotation.y = transforms[i].ry;
        msg.transforms[i].transform.rotation.z = transforms[i].rz;
        msg.transforms[i].transform.rotation.w = transforms[i].rw;
    }
    publisher->publish(msg);
}

// -----------------------------------------------------------------------------
// Twist
// -----------------------------------------------------------------------------

void PublishQrdTwistToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size != sizeof(ipc::QrdTwistData)) return;
    ipc::QrdTwistData header;
    std::memcpy(&header, data, sizeof(header));

    static std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers;
    std::string topic(header.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = header.linear_x;
    msg.linear.y = header.linear_y;
    msg.linear.z = header.linear_z;
    msg.angular.x = header.angular_x;
    msg.angular.y = header.angular_y;
    msg.angular.z = header.angular_z;
    publishers[topic]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(QRD_WRENCH_DATA, PublishQrdWrenchToROS)
CHRONO_ROS_REGISTER_HANDLER(QRD_JOINT_STATE_DATA, PublishQrdJointStateToROS)
CHRONO_ROS_REGISTER_HANDLER(QRD_ODOM_DATA, PublishQrdOdomToROS)
CHRONO_ROS_REGISTER_HANDLER(QRD_TF_DATA, PublishQrdTFToROS)
CHRONO_ROS_REGISTER_HANDLER(QRD_TWIST_DATA, PublishQrdTwistToROS)

}  // namespace ros
}  // namespace chrono

// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Subprocess (chrono_ros_node) ROS subscribers for quardet_sim, forwarding
// received data back to the main process via IPC.
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/handlers/quardet/ChROSQuardetIPC.h"

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cstring>
#include <algorithm>
#include <iostream>

namespace chrono {
namespace ros {

// -----------------------------------------------------------------------------
// JointState subscriber
// -----------------------------------------------------------------------------

static rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr g_qrd_joint_state_subscriber;
static ipc::IPCChannel* g_qrd_joint_state_channel = nullptr;

void OnQrdJointStateReceived(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!g_qrd_joint_state_channel) return;

    ipc::QrdDoubleArrayData data{};
    data.count = std::min<uint32_t>(static_cast<uint32_t>(msg->position.size()), ipc::QrdDoubleArrayData::MAX_VALUES);
    for (uint32_t i = 0; i < data.count; ++i) data.values[i] = msg->position[i];

    static ipc::Message ipc_msg;
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::QRD_JOINT_STATE_SUB, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));

    if (!g_qrd_joint_state_channel->SendMessage(ipc_msg)) {
        std::cerr << "[SUBPROCESS] Failed to send QrdJointState via IPC" << std::endl;
    }
}

void SetupQrdJointStateSubscriber(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node,
                                  ipc::IPCChannel* channel) {
    std::string topic_name = data_size > 0 ? std::string(reinterpret_cast<const char*>(data), data_size)
                                            : "/joint_states";
    g_qrd_joint_state_channel = channel;
    if (!g_qrd_joint_state_subscriber) {
        g_qrd_joint_state_subscriber =
            node->create_subscription<sensor_msgs::msg::JointState>(topic_name, 1, OnQrdJointStateReceived);
    }
}

// -----------------------------------------------------------------------------
// JointTrajectory subscriber
// -----------------------------------------------------------------------------

static rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr g_qrd_joint_traj_subscriber;
static ipc::IPCChannel* g_qrd_joint_traj_channel = nullptr;

void OnQrdJointTrajectoryReceived(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (!g_qrd_joint_traj_channel || msg->points.empty()) return;

    const auto& positions = msg->points[0].positions;
    ipc::QrdDoubleArrayData data{};
    data.count = std::min<uint32_t>(static_cast<uint32_t>(positions.size()), ipc::QrdDoubleArrayData::MAX_VALUES);
    for (uint32_t i = 0; i < data.count; ++i) data.values[i] = positions[i];

    static ipc::Message ipc_msg;
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::QRD_JOINT_TRAJECTORY_SUB, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));

    if (!g_qrd_joint_traj_channel->SendMessage(ipc_msg)) {
        std::cerr << "[SUBPROCESS] Failed to send QrdJointTrajectory via IPC" << std::endl;
    }
}

void SetupQrdJointTrajectorySubscriber(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node,
                                       ipc::IPCChannel* channel) {
    std::string topic_name = data_size > 0 ? std::string(reinterpret_cast<const char*>(data), data_size)
                                            : "/joint_group_position_controller/command";
    g_qrd_joint_traj_channel = channel;
    if (!g_qrd_joint_traj_subscriber) {
        g_qrd_joint_traj_subscriber = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            topic_name, 100, OnQrdJointTrajectoryReceived);
    }
}

// -----------------------------------------------------------------------------
// Clock subscriber
// -----------------------------------------------------------------------------

static rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr g_qrd_clock_subscriber;
static ipc::IPCChannel* g_qrd_clock_channel = nullptr;

void OnQrdClockReceived(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    if (!g_qrd_clock_channel) return;

    ipc::QrdClockSubData data{};
    data.sec = msg->clock.sec;
    data.nsec = msg->clock.nanosec;

    static ipc::Message ipc_msg;
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::QRD_CLOCK_SUB, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));

    if (!g_qrd_clock_channel->SendMessage(ipc_msg)) {
        std::cerr << "[SUBPROCESS] Failed to send QrdClock via IPC" << std::endl;
    }
}

void SetupQrdClockSubscriber(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node,
                             ipc::IPCChannel* channel) {
    std::string topic_name = data_size > 0 ? std::string(reinterpret_cast<const char*>(data), data_size) : "/clock";
    g_qrd_clock_channel = channel;
    if (!g_qrd_clock_subscriber) {
        g_qrd_clock_subscriber = node->create_subscription<rosgraph_msgs::msg::Clock>(topic_name, 1, OnQrdClockReceived);
    }
}

// -----------------------------------------------------------------------------
// TeleportPose subscriber
// -----------------------------------------------------------------------------

static rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr g_qrd_teleport_pose_subscriber;
static ipc::IPCChannel* g_qrd_teleport_pose_channel = nullptr;

void OnQrdTeleportPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!g_qrd_teleport_pose_channel) return;

    ipc::QrdPoseSubData data{};
    data.pos_x = msg->pose.position.x;
    data.pos_y = msg->pose.position.y;
    data.pos_z = msg->pose.position.z;
    data.ori_x = msg->pose.orientation.x;
    data.ori_y = msg->pose.orientation.y;
    data.ori_z = msg->pose.orientation.z;
    data.ori_w = msg->pose.orientation.w;

    static ipc::Message ipc_msg;
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::QRD_TELEPORT_POSE_SUB, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));

    if (!g_qrd_teleport_pose_channel->SendMessage(ipc_msg)) {
        std::cerr << "[SUBPROCESS] Failed to send QrdTeleportPose via IPC" << std::endl;
    }
}

void SetupQrdTeleportPoseSubscriber(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node,
                                    ipc::IPCChannel* channel) {
    std::string topic_name = data_size > 0 ? std::string(reinterpret_cast<const char*>(data), data_size)
                                            : "/quadronior/teleport_pose";
    g_qrd_teleport_pose_channel = channel;
    if (!g_qrd_teleport_pose_subscriber) {
        g_qrd_teleport_pose_subscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name, 1, OnQrdTeleportPoseReceived);
    }
}

CHRONO_ROS_REGISTER_HANDLER(QRD_JOINT_STATE_SUB, SetupQrdJointStateSubscriber)
CHRONO_ROS_REGISTER_HANDLER(QRD_JOINT_TRAJECTORY_SUB, SetupQrdJointTrajectorySubscriber)
CHRONO_ROS_REGISTER_HANDLER(QRD_CLOCK_SUB, SetupQrdClockSubscriber)
CHRONO_ROS_REGISTER_HANDLER(QRD_TELEPORT_POSE_SUB, SetupQrdTeleportPoseSubscriber)

}  // namespace ros
}  // namespace chrono

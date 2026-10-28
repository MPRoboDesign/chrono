// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// quardet_sim (chrono_ws) bidirectional subscriber ROS handlers: JointState,
// JointTrajectory, Clock. Follows the same pattern as ChROSDriverInputsHandler:
// subprocess subscribes to ROS, forwards received data to the main process via
// IPC, where HandleIncomingMessage() applies it.
//
// Implementation files:
// - ChROSQuardetSubHandlers.cpp: main process logic (HandleIncomingMessage)
// - ChROSQuardetSubHandlers_ros.cpp: subprocess ROS subscribers
// =============================================================================

#ifndef CH_ROS_QUARDET_SUB_HANDLERS_H
#define CH_ROS_QUARDET_SUB_HANDLERS_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/quardet/ChROSQuardetIPC.h"

#include <string>
#include <vector>
#include <mutex>

namespace chrono {
namespace ros {

/// Subscribes to sensor_msgs/JointState, exposing the received positions.
class CH_ROS_API ChROSQuardetJointStateSubHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetJointStateSubHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    std::vector<double> joint_val;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_JOINT_STATE_SUB; }
    virtual bool SupportsIncomingMessages() const override { return true; }
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;

  protected:
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    const std::string m_topic;
    bool m_subscriber_setup_sent = false;
    std::mutex m_mutex;
};

/// Subscribes to trajectory_msgs/JointTrajectory, exposing points[0].positions.
class CH_ROS_API ChROSQuardetJointTrajectorySubHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetJointTrajectorySubHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    std::vector<double> joint_val;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_JOINT_TRAJECTORY_SUB; }
    virtual bool SupportsIncomingMessages() const override { return true; }
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;

  protected:
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    const std::string m_topic;
    bool m_subscriber_setup_sent = false;
    std::mutex m_mutex;
};

/// Subscribes to rosgraph_msgs/Clock, exposing the received simulation time.
class CH_ROS_API ChROSQuardetClockSubHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetClockSubHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    double clock_in_sec = 0.0;
    int sec = 0;
    int nsec = 0;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_CLOCK_SUB; }
    virtual bool SupportsIncomingMessages() const override { return true; }
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;

  protected:
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    const std::string m_topic;
    bool m_subscriber_setup_sent = false;
    std::mutex m_mutex;
};

/// Subscribes to geometry_msgs/PoseStamped, exposing the last received pose
/// for the main process to apply as a rigid teleport of the robot root.
class CH_ROS_API ChROSQuardetTeleportPoseSubHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetTeleportPoseSubHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    double ori_x = 0.0, ori_y = 0.0, ori_z = 0.0, ori_w = 1.0;
    bool has_new_pose = false;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_TELEPORT_POSE_SUB; }
    virtual bool SupportsIncomingMessages() const override { return true; }
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;

  protected:
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    const std::string m_topic;
    bool m_subscriber_setup_sent = false;
    std::mutex m_mutex;
};

}  // namespace ros
}  // namespace chrono

#endif

// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// quardet_sim (chrono_ws) state-publisher ROS handlers: Wrench, JointState,
// Odometry, TF, Twist. These publish immediately (via Publish()) rather than
// on the generic per-tick GetSerializedData() poll, matching quardet_sim's
// explicit, conditional call sites.
//
// Implementation files:
// - ChROSQuardetStateHandlers.cpp: main process logic (Initialize, Publish)
// - ChROSQuardetStateHandlers_ros.cpp: subprocess ROS publishing
// =============================================================================

#ifndef CH_ROS_QUARDET_STATE_HANDLERS_H
#define CH_ROS_QUARDET_STATE_HANDLERS_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSIPCInterface.h"
#include "chrono_ros/handlers/quardet/ChROSQuardetIPC.h"

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChFrame.h"

#include <string>
#include <vector>
#include <memory>

namespace chrono {
namespace ros {

/// Publishes geometry_msgs/WrenchStamped for a single foot contact.
class CH_ROS_API ChROSQuardetWrenchHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetWrenchHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    int secs = 0;
    int nsecs = 0;
    std::string frame_id;
    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_WRENCH_DATA; }
    void Publish();

  private:
    const std::string m_topic;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

/// Publishes sensor_msgs/JointState for the 12 actuated joints.
class CH_ROS_API ChROSQuardetJointStateHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetJointStateHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    int sec = 0;
    int nsec = 0;
    std::string frame_id;
    std::vector<std::string> name;
    std::vector<double> pos = std::vector<double>(12, 0.0);
    std::vector<double> vel = std::vector<double>(12, 0.0);
    std::vector<double> eff = std::vector<double>(12, 1.0);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_JOINT_STATE_DATA; }
    void Publish();

  private:
    const std::string m_topic;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

/// Publishes nav_msgs/Odometry for the robot trunk.
class CH_ROS_API ChROSQuardetOdomHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetOdomHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    int sec = 0;
    int nsec = 0;
    std::string frame_id;
    std::string child_frame_id;
    double x_vel = 0.0, y_vel = 0.0, z_vel = 0.0;
    double x_pos = 0.0, y_pos = 0.0, z_pos = 0.0;
    double x_ori = 0.0, y_ori = 0.0, z_ori = 0.0, w_ori = 0.0;
    double alpha = 0.0, beta = 0.0, gamma = 0.0;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_ODOM_DATA; }
    void Publish();

  private:
    const std::string m_topic;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

/// Publishes the full robot TF tree (N body-to-body transforms) as tf2_msgs/TFMessage.
class CH_ROS_API ChROSQuardetTFHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetTFHandler(int num_tf_) : ChROSHandler(1), num_tf(num_tf_) {}

    std::vector<std::shared_ptr<chrono::ChBody>> parent_body;
    std::vector<std::shared_ptr<chrono::ChBody>> child_body;
    int sec = 0;
    int nsec = 0;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_TF_DATA; }
    void Publish();

  private:
    int num_tf;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

/// Publishes a single fixed TF transform (trunk -> camera_link) as tf2_msgs/TFMessage.
class CH_ROS_API ChROSQuardetCameraLinkTFHandler : public ChROSHandler {
  public:
    ChROSQuardetCameraLinkTFHandler() : ChROSHandler(1) {}

    int sec = 0;
    int nsec = 0;
    /// Camera mount pose relative to the trunk body - set to the same ChFrame
    /// the active camera sensor was constructed with.
    ChFrame<double> offset_pose;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_TF_DATA; }
    void Publish();

  private:
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

/// Publishes geometry_msgs/Twist (used for both cmd_vel velocity and robot_pose position/RPY).
class CH_ROS_API ChROSQuardetTwistHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetTwistHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic) {}

    double x = 0.0, y = 0.0, z = 0.0;
    double alpha = 0.0, beta = 0.0, gamma = 0.0;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_TWIST_DATA; }
    void Publish();

  private:
    const std::string m_topic;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

}  // namespace ros
}  // namespace chrono

#endif

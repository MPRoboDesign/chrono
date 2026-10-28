// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Main-process logic for quardet_sim state-publisher handlers.
// =============================================================================

#include "chrono_ros/handlers/quardet/ChROSQuardetStateHandlers.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include <cstring>

namespace chrono {
namespace ros {

static void CopyTruncated(char* dst, size_t dst_size, const std::string& src) {
    std::strncpy(dst, src.c_str(), dst_size - 1);
    dst[dst_size - 1] = '\0';
}

// -----------------------------------------------------------------------------
// Wrench
// -----------------------------------------------------------------------------

bool ChROSQuardetWrenchHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic)) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetWrenchHandler::Publish() {
    if (!m_ipc_interface) return;

    ipc::QrdWrenchData data{};
    CopyTruncated(data.topic_name, sizeof(data.topic_name), m_topic);
    CopyTruncated(data.frame_id, sizeof(data.frame_id), frame_id);
    data.sec = secs;
    data.nsec = static_cast<uint32_t>(nsecs);
    data.force_x = x;
    data.force_y = y;
    data.force_z = z;
    data.torque_x = roll;
    data.torque_y = pitch;
    data.torque_z = yaw;

    m_ipc_interface->SendHandlerData(GetMessageType(), &data, sizeof(data));
}

// -----------------------------------------------------------------------------
// JointState (publisher)
// -----------------------------------------------------------------------------

bool ChROSQuardetJointStateHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic)) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetJointStateHandler::Publish() {
    if (!m_ipc_interface) return;

    ipc::QrdJointStateData data{};
    CopyTruncated(data.topic_name, sizeof(data.topic_name), m_topic);
    CopyTruncated(data.frame_id, sizeof(data.frame_id), frame_id);
    data.sec = sec;
    data.nsec = static_cast<uint32_t>(nsec);

    uint32_t n = static_cast<uint32_t>(name.size());
    if (n > ipc::QrdJointStateData::MAX_JOINTS) n = ipc::QrdJointStateData::MAX_JOINTS;
    data.num_joints = n;
    for (uint32_t i = 0; i < n; ++i) {
        CopyTruncated(data.names[i], sizeof(data.names[i]), name[i]);
        data.position[i] = i < pos.size() ? pos[i] : 0.0;
        data.velocity[i] = i < vel.size() ? vel[i] : 0.0;
        data.effort[i] = i < eff.size() ? eff[i] : 0.0;
    }

    m_ipc_interface->SendHandlerData(GetMessageType(), &data, sizeof(data));
}

// -----------------------------------------------------------------------------
// Odometry
// -----------------------------------------------------------------------------

bool ChROSQuardetOdomHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic)) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetOdomHandler::Publish() {
    if (!m_ipc_interface) return;

    ipc::QrdOdomData data{};
    CopyTruncated(data.topic_name, sizeof(data.topic_name), m_topic);
    CopyTruncated(data.frame_id, sizeof(data.frame_id), frame_id);
    CopyTruncated(data.child_frame_id, sizeof(data.child_frame_id), child_frame_id);
    data.sec = sec;
    data.nsec = static_cast<uint32_t>(nsec);
    data.lin_vel_x = x_vel;
    data.lin_vel_y = y_vel;
    data.lin_vel_z = z_vel;
    data.ang_vel_x = alpha;
    data.ang_vel_y = beta;
    data.ang_vel_z = gamma;
    data.pos_x = x_pos;
    data.pos_y = y_pos;
    data.pos_z = z_pos;
    data.ori_x = x_ori;
    data.ori_y = y_ori;
    data.ori_z = z_ori;
    data.ori_w = w_ori;

    m_ipc_interface->SendHandlerData(GetMessageType(), &data, sizeof(data));
}

// -----------------------------------------------------------------------------
// TF (N transforms, robot body tree)
// -----------------------------------------------------------------------------

bool ChROSQuardetTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, "/tf")) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetTFHandler::Publish() {
    if (!m_ipc_interface) return;

    std::vector<uint8_t> buffer(sizeof(ipc::QrdTFHeader) + num_tf * sizeof(ipc::QrdTFTransform));
    auto* header = reinterpret_cast<ipc::QrdTFHeader*>(buffer.data());
    header->sec = sec;
    header->nsec = static_cast<uint32_t>(nsec);
    header->num_transforms = static_cast<uint32_t>(num_tf);

    auto* transforms = reinterpret_cast<ipc::QrdTFTransform*>(buffer.data() + sizeof(ipc::QrdTFHeader));
    for (int i = 0; i < num_tf; ++i) {
        CopyTruncated(transforms[i].parent_frame, sizeof(transforms[i].parent_frame), parent_body[i]->GetName());
        CopyTruncated(transforms[i].child_frame, sizeof(transforms[i].child_frame), child_body[i]->GetName());

        chrono::ChFrame<> child_to_parent =
            parent_body[i]->GetFrameRefToAbs().GetInverse() * child_body[i]->GetFrameRefToAbs();
        transforms[i].tx = child_to_parent.GetPos().x();
        transforms[i].ty = child_to_parent.GetPos().y();
        transforms[i].tz = child_to_parent.GetPos().z();
        transforms[i].rx = child_to_parent.GetRot().e1();
        transforms[i].ry = child_to_parent.GetRot().e2();
        transforms[i].rz = child_to_parent.GetRot().e3();
        transforms[i].rw = child_to_parent.GetRot().e0();
    }

    m_ipc_interface->SendHandlerData(GetMessageType(), buffer.data(), buffer.size());
}

// -----------------------------------------------------------------------------
// Fixed camera_link TF
// -----------------------------------------------------------------------------

bool ChROSQuardetCameraLinkTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, "/tf")) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetCameraLinkTFHandler::Publish() {
    if (!m_ipc_interface) return;

    std::vector<uint8_t> buffer(sizeof(ipc::QrdTFHeader) + sizeof(ipc::QrdTFTransform));
    auto* header = reinterpret_cast<ipc::QrdTFHeader*>(buffer.data());
    header->sec = sec;
    header->nsec = static_cast<uint32_t>(nsec);
    header->num_transforms = 1;

    auto* transform = reinterpret_cast<ipc::QrdTFTransform*>(buffer.data() + sizeof(ipc::QrdTFHeader));
    CopyTruncated(transform->parent_frame, sizeof(transform->parent_frame), "trunk");
    CopyTruncated(transform->child_frame, sizeof(transform->child_frame), "camera_link");
    // Camera mount pose relative to trunk, as set by quardet_sim from the
    // active camera's offset pose (robot.yaml cam_offset_pose/depth_cam_offset_pose).
    const ChVector3d& pos = offset_pose.GetPos();
    const ChQuaterniond& rot = offset_pose.GetRot();
    transform->tx = pos.x();
    transform->ty = pos.y();
    transform->tz = pos.z();
    transform->rx = rot.e1();
    transform->ry = rot.e2();
    transform->rz = rot.e3();
    transform->rw = rot.e0();

    m_ipc_interface->SendHandlerData(GetMessageType(), buffer.data(), buffer.size());
}

// -----------------------------------------------------------------------------
// Twist
// -----------------------------------------------------------------------------

bool ChROSQuardetTwistHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic)) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetTwistHandler::Publish() {
    if (!m_ipc_interface) return;

    ipc::QrdTwistData data{};
    CopyTruncated(data.topic_name, sizeof(data.topic_name), m_topic);
    data.linear_x = x;
    data.linear_y = y;
    data.linear_z = z;
    data.angular_x = alpha;
    data.angular_y = beta;
    data.angular_z = gamma;

    m_ipc_interface->SendHandlerData(GetMessageType(), &data, sizeof(data));
}

}  // namespace ros
}  // namespace chrono

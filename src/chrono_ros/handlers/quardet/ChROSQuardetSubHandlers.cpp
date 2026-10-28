// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Main-process logic for quardet_sim bidirectional subscriber handlers.
// =============================================================================

#include "chrono_ros/handlers/quardet/ChROSQuardetSubHandlers.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

namespace chrono {
namespace ros {

// -----------------------------------------------------------------------------
// JointState subscriber
// -----------------------------------------------------------------------------

bool ChROSQuardetJointStateSubHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    return ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic);
}

void ChROSQuardetJointStateSubHandler::HandleIncomingMessage(const ipc::Message& msg) {
    const auto* data = msg.GetPayload<ipc::QrdDoubleArrayData>();
    std::lock_guard<std::mutex> lock(m_mutex);
    joint_val.assign(data->values, data->values + data->count);
}

std::vector<uint8_t> ChROSQuardetJointStateSubHandler::GetSerializedData(double /*time*/) {
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        return std::vector<uint8_t>(m_topic.begin(), m_topic.end());
    }
    return {};
}

// -----------------------------------------------------------------------------
// JointTrajectory subscriber
// -----------------------------------------------------------------------------

bool ChROSQuardetJointTrajectorySubHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    return ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic);
}

void ChROSQuardetJointTrajectorySubHandler::HandleIncomingMessage(const ipc::Message& msg) {
    const auto* data = msg.GetPayload<ipc::QrdDoubleArrayData>();
    std::lock_guard<std::mutex> lock(m_mutex);
    joint_val.assign(data->values, data->values + data->count);
}

std::vector<uint8_t> ChROSQuardetJointTrajectorySubHandler::GetSerializedData(double /*time*/) {
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        return std::vector<uint8_t>(m_topic.begin(), m_topic.end());
    }
    return {};
}

// -----------------------------------------------------------------------------
// Clock subscriber
// -----------------------------------------------------------------------------

bool ChROSQuardetClockSubHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    return ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic);
}

void ChROSQuardetClockSubHandler::HandleIncomingMessage(const ipc::Message& msg) {
    const auto* data = msg.GetPayload<ipc::QrdClockSubData>();
    std::lock_guard<std::mutex> lock(m_mutex);
    sec = data->sec;
    nsec = static_cast<int>(data->nsec);
    clock_in_sec = static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

std::vector<uint8_t> ChROSQuardetClockSubHandler::GetSerializedData(double /*time*/) {
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        return std::vector<uint8_t>(m_topic.begin(), m_topic.end());
    }
    return {};
}

// -----------------------------------------------------------------------------
// TeleportPose subscriber
// -----------------------------------------------------------------------------

bool ChROSQuardetTeleportPoseSubHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    return ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic);
}

void ChROSQuardetTeleportPoseSubHandler::HandleIncomingMessage(const ipc::Message& msg) {
    const auto* data = msg.GetPayload<ipc::QrdPoseSubData>();
    std::lock_guard<std::mutex> lock(m_mutex);
    pos_x = data->pos_x;
    pos_y = data->pos_y;
    pos_z = data->pos_z;
    ori_x = data->ori_x;
    ori_y = data->ori_y;
    ori_z = data->ori_z;
    ori_w = data->ori_w;
    has_new_pose = true;
}

std::vector<uint8_t> ChROSQuardetTeleportPoseSubHandler::GetSerializedData(double /*time*/) {
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        return std::vector<uint8_t>(m_topic.begin(), m_topic.end());
    }
    return {};
}

}  // namespace ros
}  // namespace chrono

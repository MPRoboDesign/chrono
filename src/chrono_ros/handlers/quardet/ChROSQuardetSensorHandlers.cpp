// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Main-process logic for quardet_sim sensor-publisher handlers.
// =============================================================================

#include "chrono_ros/handlers/quardet/ChROSQuardetSensorHandlers.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono/utils/ChConstants.h"

#include <cmath>
#include <cstring>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

static void CopyTruncated(char* dst, size_t dst_size, const std::string& src) {
    std::strncpy(dst, src.c_str(), dst_size - 1);
    dst[dst_size - 1] = '\0';
}

// -----------------------------------------------------------------------------
// Image
// -----------------------------------------------------------------------------

bool ChROSQuardetImageHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic)) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetImageHandler::Publish() {
    if (!m_ipc_interface || !cam) return;

    auto rgba_ptr = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    if (!rgba_ptr->Buffer) return;

    unsigned int cam_height = rgba_ptr->Height;
    unsigned int cam_width = rgba_ptr->Width;
    uint32_t step = cam_width * 4;
    size_t image_size = static_cast<size_t>(step) * cam_height;

    std::vector<uint8_t> buffer(sizeof(ipc::QrdImageHeader) + image_size);
    auto* header = reinterpret_cast<ipc::QrdImageHeader*>(buffer.data());
    CopyTruncated(header->topic_name, sizeof(header->topic_name), topic);
    CopyTruncated(header->frame_id, sizeof(header->frame_id), "camera_link");
    header->sec = sec;
    header->nsec = static_cast<uint32_t>(nsec);
    header->width = cam_width;
    header->height = cam_height;
    header->step = step;

    uint8_t* pixel_data = buffer.data() + sizeof(ipc::QrdImageHeader);

    // Same mirror + reverse packing as the original ImagePub implementation.
    for (unsigned int y = 0; y < cam_height; ++y) {
        for (unsigned int x = 0; x < cam_width; ++x) {
            unsigned int idx = y * cam_width + x;
            PixelRGBA8 pixel = rgba_ptr->Buffer[idx];

            unsigned int mirror_x = cam_width - 1 - x;
            unsigned int mirror_idx = y * cam_width + mirror_x;

            long total_size = static_cast<long>(cam_width) * cam_height * 4 - 1;
            long data_idx = total_size - static_cast<long>(mirror_idx) * 4;

            pixel_data[data_idx - 3] = pixel.R;
            pixel_data[data_idx - 2] = pixel.G;
            pixel_data[data_idx - 1] = pixel.B;
            pixel_data[data_idx - 0] = pixel.A;
        }
    }

    m_ipc_interface->SendHandlerData(GetMessageType(), buffer.data(), buffer.size());
}

// -----------------------------------------------------------------------------
// CameraInfo
// -----------------------------------------------------------------------------

bool ChROSQuardetCameraInfoHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic)) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetCameraInfoHandler::Publish() {
    if (!m_ipc_interface || !cam) return;

    ipc::QrdCameraInfoData data{};
    CopyTruncated(data.topic_name, sizeof(data.topic_name), topic);
    CopyTruncated(data.frame_id, sizeof(data.frame_id), "camera_link");
    data.sec = sec;
    data.nsec = static_cast<uint32_t>(nsec);

    auto rgba_ptr = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    data.height = rgba_ptr->Height;
    data.width = rgba_ptr->Width;

    auto dist = cam->GetCameraDistortionCoefficients();
    data.d[0] = dist.x();
    data.d[1] = dist.y();
    data.d[2] = 0.0;
    data.d[3] = 0.0;
    data.d[4] = dist.z();

    auto K = cam->GetCameraIntrinsicMatrix();
    data.k[0] = K.GetAxisX().x(); data.k[1] = K.GetAxisY().x(); data.k[2] = K.GetAxisZ().x();
    data.k[3] = K.GetAxisX().y(); data.k[4] = K.GetAxisY().y(); data.k[5] = K.GetAxisZ().y();
    data.k[6] = K.GetAxisX().z(); data.k[7] = K.GetAxisY().z(); data.k[8] = K.GetAxisZ().z();

    data.r[0] = 1.0; data.r[1] = 0.0; data.r[2] = 0.0;
    data.r[3] = 0.0; data.r[4] = 1.0; data.r[5] = 0.0;
    data.r[6] = 0.0; data.r[7] = 0.0; data.r[8] = 1.0;

    data.p[0] = K.GetAxisX().x(); data.p[1] = K.GetAxisY().x(); data.p[2] = K.GetAxisZ().x(); data.p[3] = 0.0;
    data.p[4] = K.GetAxisX().y(); data.p[5] = K.GetAxisY().y(); data.p[6] = K.GetAxisZ().y(); data.p[7] = 0.0;
    data.p[8] = K.GetAxisX().z(); data.p[9] = K.GetAxisY().z(); data.p[10] = K.GetAxisZ().z(); data.p[11] = 0.0;

    m_ipc_interface->SendHandlerData(GetMessageType(), &data, sizeof(data));
}

// -----------------------------------------------------------------------------
// PointCloud (depth)
// -----------------------------------------------------------------------------

bool ChROSQuardetPointCloudHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic)) {
        return false;
    }
    m_ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(interface);
    return true;
}

void ChROSQuardetPointCloudHandler::Publish() {
    if (!m_ipc_interface || !depth_cam) return;

    auto depth_ptr = depth_cam->GetMostRecentBuffer<UserDepthBufferPtr>();
    if (!depth_ptr->Buffer) return;

    unsigned int cam_height = depth_ptr->Height;
    unsigned int cam_width = depth_ptr->Width;

    // Invert __raygen__depth_camera (chrono_sensor/optix/shaders/depth_cam_raygen.cu,
    // PINHOLE lens - ChDepthCamera's default) exactly, rather than assuming a
    // textbook pinhole model:
    //   - row 0 of the buffer is the BOTTOM of the image (d.y = -1 looks down),
    //     so up-offset grows with v, not against it;
    //   - pixels are square: d.y is pre-scaled by H/W, sharing one h_factor
    //     with d.x;
    //   - the stored value is the Euclidean ray length, not the depth along
    //     the optical axis, so each point is range * normalized ray direction.
    // Frame matches camera_link: X forward, Y left, Z up.
    const float h_factor = img_fov / static_cast<float>(CH_PI) * 2.0f;
    const float aspect = static_cast<float>(cam_height) / static_cast<float>(cam_width);
    const float max_depth = depth_cam->GetMaxDepth();

    std::vector<float> points;
    points.reserve(static_cast<size_t>(cam_width) * cam_height * 3);

    for (unsigned int v = 0; v < cam_height; ++v) {
        const float dy = ((static_cast<float>(v) + 0.5f) / cam_height * 2.0f - 1.0f) * aspect;
        for (unsigned int u = 0; u < cam_width; ++u) {
            float depth = depth_ptr->Buffer[v * cam_width + u].depth;
            // Misses are written as max_depth (miss.cu) - not a real surface.
            if (depth <= 0.0f || depth >= max_depth || std::isinf(depth) || std::isnan(depth)) continue;

            const float dx = (static_cast<float>(u) + 0.5f) / cam_width * 2.0f - 1.0f;
            const float ray_x = 1.0f;
            const float ray_y = -dx * h_factor;
            const float ray_z = dy * h_factor;
            const float scale = depth / std::sqrt(ray_x * ray_x + ray_y * ray_y + ray_z * ray_z);

            points.push_back(ray_x * scale);
            points.push_back(ray_y * scale);
            points.push_back(ray_z * scale);
        }
    }

    uint32_t num_points = static_cast<uint32_t>(points.size() / 3);
    size_t points_bytes = points.size() * sizeof(float);

    std::vector<uint8_t> buffer(sizeof(ipc::QrdPointCloudHeader) + points_bytes);
    auto* header = reinterpret_cast<ipc::QrdPointCloudHeader*>(buffer.data());
    CopyTruncated(header->topic_name, sizeof(header->topic_name), topic);
    CopyTruncated(header->frame_id, sizeof(header->frame_id), "camera_link");
    header->sec = sec;
    header->nsec = static_cast<uint32_t>(nsec);
    header->num_points = num_points;

    std::memcpy(buffer.data() + sizeof(ipc::QrdPointCloudHeader), points.data(), points_bytes);

    m_ipc_interface->SendHandlerData(GetMessageType(), buffer.data(), buffer.size());
}

}  // namespace ros
}  // namespace chrono

// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// quardet_sim (chrono_ws) sensor-publisher ROS handlers: Image, CameraInfo,
// PointCloud2 (depth). Publish immediately via Publish(), matching
// quardet_sim's explicit call sites.
//
// Implementation files:
// - ChROSQuardetSensorHandlers.cpp: main process logic (reads Chrono sensor buffers)
// - ChROSQuardetSensorHandlers_ros.cpp: subprocess ROS publishing
// =============================================================================

#ifndef CH_ROS_QUARDET_SENSOR_HANDLERS_H
#define CH_ROS_QUARDET_SENSOR_HANDLERS_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSIPCInterface.h"
#include "chrono_ros/handlers/quardet/ChROSQuardetIPC.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"

#include <string>
#include <memory>

namespace chrono {
namespace ros {

/// Publishes sensor_msgs/Image (RGBA8) from a Chrono RGB camera buffer.
class CH_ROS_API ChROSQuardetImageHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetImageHandler(const std::string& topic_) : ChROSHandler(1), topic(topic_) {}

    int sec = 0;
    int nsec = 0;
    std::shared_ptr<chrono::sensor::ChCameraSensor> cam;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_IMAGE_DATA; }
    void Publish();

  private:
    const std::string topic;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

/// Publishes sensor_msgs/CameraInfo for the RGB camera.
class CH_ROS_API ChROSQuardetCameraInfoHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetCameraInfoHandler(const std::string& topic_) : ChROSHandler(1), topic(topic_) {}

    int sec = 0;
    int nsec = 0;
    std::shared_ptr<chrono::sensor::ChCameraSensor> cam;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_CAMERA_INFO_DATA; }
    void Publish();

  private:
    const std::string topic;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

/// Publishes sensor_msgs/PointCloud2 from a Chrono depth camera buffer.
class CH_ROS_API ChROSQuardetPointCloudHandler : public ChROSHandler {
  public:
    explicit ChROSQuardetPointCloudHandler(const std::string& topic_) : ChROSHandler(1), topic(topic_) {}

    int sec = 0;
    int nsec = 0;
    std::shared_ptr<chrono::sensor::ChDepthCamera> depth_cam;
    unsigned int img_width = 0;
    unsigned int img_height = 0;
    float img_fov = 0.0f;

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::QRD_POINTCLOUD_DATA; }
    void Publish();

  private:
    const std::string topic;
    std::shared_ptr<ChROSIPCInterface> m_ipc_interface;
};

}  // namespace ros
}  // namespace chrono

#endif

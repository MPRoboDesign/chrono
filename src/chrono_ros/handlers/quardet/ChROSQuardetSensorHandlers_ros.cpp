// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Subprocess (chrono_ros_node) ROS publishing for quardet_sim sensor handlers.
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/quardet/ChROSQuardetIPC.h"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

// -----------------------------------------------------------------------------
// Image
// -----------------------------------------------------------------------------

void PublishQrdImageToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size < sizeof(ipc::QrdImageHeader)) return;
    ipc::QrdImageHeader header;
    std::memcpy(&header, data, sizeof(header));

    size_t expected = sizeof(ipc::QrdImageHeader) + static_cast<size_t>(header.step) * header.height;
    if (data_size != expected) return;

    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers;
    std::string topic(header.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<sensor_msgs::msg::Image>(topic, 10);
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp.sec = header.sec;
    msg.header.stamp.nanosec = header.nsec;
    msg.header.frame_id = std::string(header.frame_id);
    msg.height = header.height;
    msg.width = header.width;
    msg.encoding = sensor_msgs::image_encodings::RGBA8;
    msg.is_bigendian = false;
    msg.step = header.step;

    size_t image_size = static_cast<size_t>(header.step) * header.height;
    msg.data.resize(image_size);
    std::memcpy(msg.data.data(), data + sizeof(ipc::QrdImageHeader), image_size);

    publishers[topic]->publish(msg);
}

// -----------------------------------------------------------------------------
// CameraInfo
// -----------------------------------------------------------------------------

void PublishQrdCameraInfoToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size != sizeof(ipc::QrdCameraInfoData)) return;
    ipc::QrdCameraInfoData header;
    std::memcpy(&header, data, sizeof(header));

    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> publishers;
    std::string topic(header.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic, 10);
    }

    sensor_msgs::msg::CameraInfo msg;
    msg.header.stamp.sec = header.sec;
    msg.header.stamp.nanosec = header.nsec;
    msg.header.frame_id = std::string(header.frame_id);
    msg.height = header.height;
    msg.width = header.width;
    msg.distortion_model = "none";
    for (int i = 0; i < 5; ++i) msg.d.push_back(header.d[i]);
    for (int i = 0; i < 9; ++i) msg.k[i] = header.k[i];
    for (int i = 0; i < 9; ++i) msg.r[i] = header.r[i];
    for (int i = 0; i < 12; ++i) msg.p[i] = header.p[i];

    publishers[topic]->publish(msg);
}

// -----------------------------------------------------------------------------
// PointCloud2 (depth)
// -----------------------------------------------------------------------------

void PublishQrdPointCloudToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel*) {
    if (data_size < sizeof(ipc::QrdPointCloudHeader)) return;
    ipc::QrdPointCloudHeader header;
    std::memcpy(&header, data, sizeof(header));

    size_t expected = sizeof(ipc::QrdPointCloudHeader) + static_cast<size_t>(header.num_points) * 3 * sizeof(float);
    if (data_size != expected) return;

    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers;
    std::string topic(header.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = header.sec;
    msg.header.stamp.nanosec = header.nsec;
    msg.header.frame_id = std::string(header.frame_id);
    msg.height = 1;
    msg.is_bigendian = false;
    msg.is_dense = false;

    sensor_msgs::msg::PointField field_x, field_y, field_z;
    field_x.name = "x"; field_x.offset = 0;  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32; field_x.count = 1;
    field_y.name = "y"; field_y.offset = 4;  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32; field_y.count = 1;
    field_z.name = "z"; field_z.offset = 8;  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32; field_z.count = 1;
    msg.fields = {field_x, field_y, field_z};
    msg.point_step = 12;

    msg.width = header.num_points;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);
    std::memcpy(msg.data.data(), data + sizeof(ipc::QrdPointCloudHeader), msg.row_step);

    publishers[topic]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(QRD_IMAGE_DATA, PublishQrdImageToROS)
CHRONO_ROS_REGISTER_HANDLER(QRD_CAMERA_INFO_DATA, PublishQrdCameraInfoToROS)
CHRONO_ROS_REGISTER_HANDLER(QRD_POINTCLOUD_DATA, PublishQrdPointCloudToROS)

}  // namespace ros
}  // namespace chrono

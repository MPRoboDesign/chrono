// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// IPC data structures for quardet_sim (chrono_ws) ROS handlers.
// Plain C++ types only - no ROS or Chrono dependencies - safe to include in
// both the main process and the chrono_ros_node subprocess.
// =============================================================================

#ifndef CH_ROS_QUARDET_IPC_H
#define CH_ROS_QUARDET_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

/// geometry_msgs/WrenchStamped - topic-multiplexed (one handler per foot)
struct QrdWrenchData {
    char topic_name[64];
    char frame_id[32];
    int32_t sec;
    uint32_t nsec;
    double force_x, force_y, force_z;
    double torque_x, torque_y, torque_z;
};

/// sensor_msgs/JointState publisher - fixed at 12 joints (matches quardet's motor count)
struct QrdJointStateData {
    static constexpr uint32_t MAX_JOINTS = 12;
    char topic_name[64];
    char frame_id[32];
    int32_t sec;
    uint32_t nsec;
    uint32_t num_joints;
    char names[MAX_JOINTS][32];
    double position[MAX_JOINTS];
    double velocity[MAX_JOINTS];
    double effort[MAX_JOINTS];
};

/// nav_msgs/Odometry
struct QrdOdomData {
    char topic_name[64];
    char frame_id[32];
    char child_frame_id[32];
    int32_t sec;
    uint32_t nsec;
    double lin_vel_x, lin_vel_y, lin_vel_z;
    double ang_vel_x, ang_vel_y, ang_vel_z;
    double pos_x, pos_y, pos_z;
    double ori_x, ori_y, ori_z, ori_w;
};

/// A single transform within a tf2_msgs/TFMessage
struct QrdTFTransform {
    char parent_frame[32];
    char child_frame[32];
    double tx, ty, tz;
    double rx, ry, rz, rw;
};

/// tf2_msgs/TFMessage header - followed by num_transforms QrdTFTransform entries
struct QrdTFHeader {
    int32_t sec;
    uint32_t nsec;
    uint32_t num_transforms;
};

/// geometry_msgs/Twist - topic-multiplexed (cmd_vel, robot_pose, ...)
struct QrdTwistData {
    char topic_name[64];
    double linear_x, linear_y, linear_z;
    double angular_x, angular_y, angular_z;
};

/// sensor_msgs/Image (RGBA8) header - followed by width*height*4 bytes of pixel data
struct QrdImageHeader {
    char topic_name[64];
    char frame_id[32];
    int32_t sec;
    uint32_t nsec;
    uint32_t width;
    uint32_t height;
    uint32_t step;
};

/// sensor_msgs/CameraInfo
struct QrdCameraInfoData {
    char topic_name[64];
    char frame_id[32];
    int32_t sec;
    uint32_t nsec;
    uint32_t width, height;
    double d[5];
    double k[9];
    double r[9];
    double p[12];
};

/// sensor_msgs/PointCloud2 header - followed by num_points * 3 floats (x,y,z)
struct QrdPointCloudHeader {
    char topic_name[64];
    char frame_id[32];
    int32_t sec;
    uint32_t nsec;
    uint32_t num_points;
};

/// Generic array-of-doubles payload, shared by JointState and JointTrajectory subscribers
struct QrdDoubleArrayData {
    static constexpr uint32_t MAX_VALUES = 16;
    uint32_t count;
    double values[MAX_VALUES];
};

/// rosgraph_msgs/Clock subscriber payload
struct QrdClockSubData {
    int32_t sec;
    uint32_t nsec;
};

/// geometry_msgs/PoseStamped subscriber payload - teleports the robot root
struct QrdPoseSubData {
    double pos_x, pos_y, pos_z;
    double ori_x, ori_y, ori_z, ori_w;
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif

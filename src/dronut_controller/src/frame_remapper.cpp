#include "frame_remapper.hpp"

FrameRemapper::FrameRemapper() : Node("frame_remapper") {
    // Initialize the transform buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to the PointCloud2 topic
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ircam1/points", 10,
        std::bind(&FrameRemapper::callback, this, std::placeholders::_1)
    );

    // Publisher for transformed PointCloud2
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ircam1/points_fixed", 10);

    RCLCPP_INFO(this->get_logger(), "FrameRemapper node started.");
}

void FrameRemapper::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
        // Lookup the transform from camera_infra1_optical_frame to world
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(
                "world", // Target frame
                "camera_infra1_optical_frame", // Source frame
                tf2::TimePointZero
            );

        // Transform the PointCloud2 message
        sensor_msgs::msg::PointCloud2 transformed_msg;
        tf2::doTransform(*msg, transformed_msg, transform_stamped);

        // Publish the transformed point cloud
        pointcloud_pub_->publish(transformed_msg);
        RCLCPP_INFO(this->get_logger(), "Published transformed PointCloud2 in the world frame.");

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
}


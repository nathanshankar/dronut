#ifndef FRAME_REMAPPER_HPP
#define FRAME_REMAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // For transforming PointCloud2
#include <geometry_msgs/msg/transform_stamped.hpp>

class FrameRemapper : public rclcpp::Node {
public:
    FrameRemapper();
    
private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ROS 2 Components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // FRAME_REMAPPER_HPP

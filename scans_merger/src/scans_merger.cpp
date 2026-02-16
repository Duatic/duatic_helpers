// Copyright (c) Ibrahim Hroob
// Licensed under the Apache License, Version 2.0
// (2025) Updated to ROS 2 Rolling/Jazzy by Marc Blöchlinger

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

// Define a consistent Point Type
typedef pcl::PointXYZI PointType;

class CloudMergerNode : public rclcpp::Node
{
public:
    CloudMergerNode() : Node("cloud_merger_node")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("destination_frame", "base_link");
        this->declare_parameter<std::string>("input_cloud_1", "/front_lidar/points");
        this->declare_parameter<std::string>("input_cloud_2", "/back_lidar/points");
        this->declare_parameter<std::string>("merged_cloud", "/merged_cloud");

        // Get parameter values
        destination_frame_ = this->get_parameter("destination_frame").as_string();
        input_cloud_1_ = this->get_parameter("input_cloud_1").as_string();
        input_cloud_2_ = this->get_parameter("input_cloud_2").as_string();
        merged_cloud_ = this->get_parameter("merged_cloud").as_string();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded: destination_frame=%s, input_cloud_1=%s, input_cloud_2=%s, merged_cloud=%s",  // NOLINT(whitespace/line_length)
                     destination_frame_.c_str(), input_cloud_1_.c_str(), input_cloud_2_.c_str(), merged_cloud_.c_str());

        // QoS Setup
        rclcpp::QoS qos_sub = rclcpp::QoS(rclcpp::SensorDataQoS());

#ifdef ROS_DISTRO_ROLLING
        // Rolling/Kilted API: Pass rclcpp::QoS object directly
        cloud_sub_1_.subscribe(this, input_cloud_1_, qos_sub);
        cloud_sub_2_.subscribe(this, input_cloud_2_, qos_sub);
#else
        // Jazzy API: Pass rmw_qos_profile_t struct
        cloud_sub_1_.subscribe(this, input_cloud_1_, qos_sub.get_rmw_qos_profile());
        cloud_sub_2_.subscribe(this, input_cloud_2_, qos_sub.get_rmw_qos_profile());
#endif

        sync_ = std::make_shared<Sync>(SyncPolicy(10), cloud_sub_1_, cloud_sub_2_);
        sync_->registerCallback(std::bind(&CloudMergerNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));  // NOLINT(whitespace/line_length)

        RCLCPP_INFO(this->get_logger(), "Cloud subscriptions set up.");

        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.best_effort();
        merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_cloud_, qos);

        // Initialize the TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Cloud merger node initialized.");
    }

private:
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud1,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud2)
    {
        sensor_msgs::msg::PointCloud2 transformed_cloud_1;
        sensor_msgs::msg::PointCloud2 transformed_cloud_2;

        auto start = this->get_clock()->now();

        // Check transforms
        if (!transformCloudToTargetFrame(*cloud1, transformed_cloud_1) ||
            !transformCloudToTargetFrame(*cloud2, transformed_cloud_2))
        {
            RCLCPP_WARN(this->get_logger(), "Cloud transformation failed.");
            return;
        }

        // Convert to PCL
        pcl::PointCloud<PointType> pcl_cloud_1;
        pcl::PointCloud<PointType> pcl_cloud_2;
        pcl::fromROSMsg(transformed_cloud_1, pcl_cloud_1);
        pcl::fromROSMsg(transformed_cloud_2, pcl_cloud_2);

        // Merge
        pcl::PointCloud<PointType> merged_cloud = pcl_cloud_1 + pcl_cloud_2;

        // Convert back to ROS
        sensor_msgs::msg::PointCloud2 output_cloud;
        pcl::toROSMsg(merged_cloud, output_cloud);
        output_cloud.header.frame_id = destination_frame_;
        output_cloud.header.stamp = transformed_cloud_1.header.stamp;

        merged_cloud_pub_->publish(std::move(output_cloud));

        auto end = this->get_clock()->now();

        RCLCPP_DEBUG(this->get_logger(),
            "Merged | %s: %zu, %s: %zu, total: %zu | Time: %f sec",
                input_cloud_1_.c_str(), pcl_cloud_1.size(),
                input_cloud_2_.c_str(), pcl_cloud_2.size(),
                merged_cloud.size(), (end - start).seconds()
            );
    }

    bool transformCloudToTargetFrame(const sensor_msgs::msg::PointCloud2 &input_cloud,
                                     sensor_msgs::msg::PointCloud2 &output_cloud)
    {
        // if already in target frame, just copy
        if (input_cloud.header.frame_id == destination_frame_) {
            output_cloud = input_cloud;
            return true;
        }

        try
        {
            if (!tf_buffer_->canTransform(destination_frame_, input_cloud.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.05)))  // NOLINT(whitespace/line_length)
            {
                return false;
            }
            // Use pcl_ros helper for easier transformation
            pcl_ros::transformPointCloud(destination_frame_, input_cloud, output_cloud, *tf_buffer_);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return false;
        }
    }

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_1_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_2_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;  // NOLINT(whitespace/line_length)
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string destination_frame_;
    std::string input_cloud_1_;
    std::string input_cloud_2_;
    std::string merged_cloud_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudMergerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

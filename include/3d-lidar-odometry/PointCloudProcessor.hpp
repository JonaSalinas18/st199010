#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Geometry>

class PointCloudProcessor : public rclcpp::Node
{
    public:
        PointCloudProcessor();

    private:
        void PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);
        void SetupParameters();
        void ExtractPointsFromNewPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);

        int KeyFrameCounter;
        int KeyFrameStepsUpdateThreshold;       //poner como parametro extra?
        int MinRange;
        int MaxRange;
        int IntensityThreshold;
        bool IsFirstIteration;
        Eigen::Isometry3f T_odometry_current;
        Eigen::Isometry3f T_odometry_keyframe;
        std::vector<Eigen::Vector3d> NewPointCloudReceived;
        std::vector<Eigen::Vector3d> KeyFramePointCloud;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr SubscriptionPtr;
};
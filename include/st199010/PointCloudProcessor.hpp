#pragma once

#include "st199010/KdTree.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>

class KdTree;

class PointCloudProcessor : public rclcpp::Node
{
    public:
        PointCloudProcessor();

    private:
        void PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);
        void SetupParameters();
        void ExtractPointsFromNewPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);
        void IterativeClosestPoint(const std::vector<Eigen::Vector3f>& newPointCloudPoints);
        void PublishTransform(const rclcpp::Time& timestamp, const std::string& frame_id);
        Eigen::Matrix3f GetSkewMatrix(const Eigen::Vector3f& point);
        Eigen::Matrix3f ComputeExpSO3(const Eigen::Vector3f& rotationVector);

        int KeyFrameCounter;
        int KeyFrameStepsUpdateThreshold;       //poner como parametro extra?
        int MinRange;
        int MaxRange;
        int IntensityThreshold;
        int MaximumNeighborDistanceThreshold;
        bool IsFirstIteration;
        Eigen::Isometry3f T_keyframe_current;
        Eigen::Isometry3f T_odometry_current;
        Eigen::Isometry3f T_odometry_keyframe;
        std::vector<Eigen::Vector3f> NewPointCloudReceived;
        std::vector<Eigen::Vector3f> KeyFramePointCloud;
        KdTree KdTreeInstance;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr SubscriptionPtr;
        std::unique_ptr<tf2_ros::TransformBroadcaster> TfBroadcaster;
};
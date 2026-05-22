#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Geometry>

using std::placeholders::_1;

namespace
{
    constexpr int MIN_RANGE_DEFAULT = 5;
    constexpr int MAX_RANGE_DEFAULT = 100;
    constexpr int INTENSITY_THRESHOLD_DEFAULT = 9000;
}

class PointCloudProcessor : public rclcpp::Node
{
    public:
        PointCloudProcessor()
        : Node("point_cloud_processor_node")
        , KeyFrameCounter(0)
        , KeyFrameStepsUpdateThreshold(3)
        , MinRange(MIN_RANGE_DEFAULT)
        , MaxRange(MAX_RANGE_DEFAULT)
        , IntensityThreshold(INTENSITY_THRESHOLD_DEFAULT)
        , IsFirstIteration(true)
        , T_odometry_current(Eigen::Isometry3f::Identity())
        , T_odometry_keyframe(Eigen::Isometry3f::Identity())
        , NewPointCloudReceived()
        , KeyFramePointCloud()
        {
            SetupParameters();

            SubscriptionPtr = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::PointCloud_Callback, this, _1));
        
            
        }

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

void PointCloudProcessor::PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg)
{
    ExtractPointsFromNewPointCloud(pointCloudMsg);

    if(IsFirstIteration)
    {
        KeyFramePointCloud = NewPointCloudReceived;
        //KdTree.buildTree(KeyFramePointCloud);
        T_odometry_keyframe = Eigen::Isometry3f::Identity();

        IsFirstIteration = false;
    }
    else
    {
        //KeyFrameCounter++;
    }
    
    RCLCPP_INFO(this->get_logger(), "Recibi nueva nube de puntos. MinRange: %d, MaxRange: %d, IntensityThreshold: %d", MinRange, MaxRange, IntensityThreshold);
}

void PointCloudProcessor::SetupParameters()
{
    this->declare_parameter<int>("minRange", MIN_RANGE_DEFAULT);
    this->declare_parameter<int>("maxRange", MAX_RANGE_DEFAULT);
    this->declare_parameter<int>("intensityThreshold", INTENSITY_THRESHOLD_DEFAULT);
    //this->declare_parameter<int>("neighborDistanceThreshold", 2);

    this->get_parameter("minRange", MinRange);
    this->get_parameter("maxRange", MaxRange);
    this->get_parameter("intensityThreshold", IntensityThreshold);
    //this->get_parameter("neighborDistanceThreshold", NeighborDistanceThreshold);
}

void PointCloudProcessor::ExtractPointsFromNewPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg)
{
    NewPointCloudReceived.clear();
    //NewPointCloudReceived.reserve(pointCloudMsg->width * pointCloudMsg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointCloudMsg, "x");       //Meter esto en una funcion?
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointCloudMsg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointCloudMsg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*pointCloudMsg, "intensity");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
    {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        float intensity = *iter_intensity;

        if(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
        {
            float rangeSquared = x*x + y*y + z*z;

            if((MinRange*MinRange < rangeSquared) && (rangeSquared < MaxRange*MaxRange) && (IntensityThreshold < intensity))
            {
                Eigen::Vector3d newPoint(x, y, z);
                NewPointCloudReceived.push_back(newPoint);
            }
        }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}

/*
    Probar:
        -Que la subsripcion este correcta.


*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Geometry>

using std::placeholders::_1;

class PointCloudProcessor : public rclcpp::Node
{
    public: PointCloudProcessor() : Node("point_cloud_processor_node")
    {
        DeclareParameters();

        SubscriptionPtr = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::PointCloud_Callback, this, _1));
    
        KeyFrameCounter = 0;
        KeyFrameStepsUpdateThreshold = 3;
        IsFirstIteration = true;  //Inicializar de otro modo?
        T_odometry_current = Eigen::Isometry3f::Identity();
        T_odometry_keyframe = Eigen::Isometry3f::Identity();

        NewPointCloudReceived = std::vector<Eigen::Vector3d>();
        KeyFramePointCloud = std::vector<Eigen::Vector3d>();
    }

    private:
        void PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);
        void DeclareParameters();

        Eigen::Isometry3f T_odometry_current;
        Eigen::Isometry3f T_odometry_keyframe;
        std::vector<Eigen::Vector3d> KeyFramePointCloud;
        std::vector<Eigen::Vector3d> NewPointCloudReceived;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr SubscriptionPtr;
        int KeyFrameCounter;
        int KeyFrameStepsUpdateThreshold;       //poner como parametro extra?
        int MinRange;
        int MaxRange;
        int IntensityThreshold;
        bool IsFirstIteration;
};

void PointCloudProcessor::PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg)
{
    NewPointCloudReceived.clear();
    //NewPointCloudReceived.reserve(pointCloudMsg->width * pointCloudMsg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointCloudMsg, "x");       //Meter esto en una funcion?
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointCloudMsg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointCloudMsg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*pointCloudMsg, "intensity");   //verificar este campo con el comando: 

    /*
    Para verificar qué campos tiene tu nube antes de correr el código, puedes hacer esto en terminal:
        ros2 topic echo /tu_topic --no-arr | head -30
    o
        ros2 topic echo /tu_topic --field fields
    
    */



    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
    {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        float intensity = *iter_intensity;

        if(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
        {
            float rangeSquared = x*x + y*y + z*z;

            if((MinRange*MinRange  < rangeSquared)      && 
               (rangeSquared       < MaxRange*MaxRange) && 
               (IntensityThreshold < intensity))
            {
                Eigen::Vector3d newPoint(x, y, z);
                NewPointCloudReceived.push_back(newPoint);
            }
        }

        //RCLCPP_INFO(this->get_logger(), "Point: x = %f, y = %f, z = %f, intensity = %f", x, y, z, intensity);
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", pointCloudMsg.data.c_str());
    }

    /* 
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
        */

    RCLCPP_INFO(this->get_logger(), "Recibi nueva nube de puntos. MinRange: %d, MaxRange: %d, IntensityThreshold: %d", MinRange, MaxRange, IntensityThreshold);
}

void PointCloudProcessor::DeclareParameters()
{
    this->declare_parameter<int>("minRange", 5);
    this->declare_parameter<int>("maxRange", 100);
    this->declare_parameter<int>("intensityThreshold", 9000);

    this->get_parameter("minRange", MinRange);
    this->get_parameter("maxRange", MaxRange);
    this->get_parameter("intensityThreshold", IntensityThreshold);
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
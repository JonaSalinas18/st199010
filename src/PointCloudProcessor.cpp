#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class PointCloudProcessor : public rclcpp::Node
{
    public: PointCloudProcessor() : Node("point_cloud_processor_node")
    {
        SubscriptionPtr = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointCloud_topic", 10, std::bind(&PointCloudProcessor::PointCloud_Callback, this, _1));
    }

    private:
        void PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg) const;
  
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr SubscriptionPtr;
};

void PointCloudProcessor::PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg) const
{
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointCloudMsg, "x");
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
        //float intensity = *iter_intensity;

        if(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
        {
            //float rangeSquared = x*x + y*y + z*z;


        }

        //RCLCPP_INFO(this->get_logger(), "Point: x = %f, y = %f, z = %f, intensity = %f", x, y, z, intensity);
    }

    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", pointCloudMsg.data.c_str());
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
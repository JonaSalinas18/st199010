#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class PointCloudProcessor : public rclcpp::Node
{
    public: PointCloudProcessor() : Node("point_cloud_processor_subscriber")
    {
        SubscriptionPtr = this->create_subscription<std_msgs::msg::PointCloud2>("pointCloud_topic", 10, std::bind(&PointCloudProcessor::PointCloud_Callback, this, _1));
    }

    private:
        void PointCloud_Callback(const std_msgs::msg::PointCloud2::SharedPtr pointCloudMsg) const
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointCloudMsg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointCloudMsg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointCloudMsg, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                float x = *iter_x;
                float y = *iter_y;
                float z = *iter_z;
                
                //RCLCPP_INFO(this->get_logger(), "Point: x = %f, y = %f, z = %f", x, y, z);
            }

            //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", pointCloudMsg.data.c_str());
        }
  
        rclcpp::Subscription<std_msgs::msg::PointCloud2>::SharedPtr SubscriptionPtr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}

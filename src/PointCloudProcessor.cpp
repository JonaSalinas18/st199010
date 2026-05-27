#include "3d-lidar-odometry/KdTree.hpp"
#include "3d-lidar-odometry/PointCloudProcessor.hpp"

using std::placeholders::_1;

namespace
{
    constexpr int MIN_RANGE_DEFAULT = 5;
    constexpr int MAX_RANGE_DEFAULT = 100;
    constexpr int INTENSITY_THRESHOLD_DEFAULT = 9000;
    constexpr int MAXIMUM_NEIGHBOR_DISTANCE_THRESHOLD_DEFAULT = 2;
}

PointCloudProcessor:: PointCloudProcessor()
: Node("point_cloud_processor_node")
, KeyFrameCounter(0)
, KeyFrameStepsUpdateThreshold(3)
, MinRange(MIN_RANGE_DEFAULT)
, MaxRange(MAX_RANGE_DEFAULT)
, IntensityThreshold(INTENSITY_THRESHOLD_DEFAULT)
, MaximumNeighborDistanceThreshold(MAXIMUM_NEIGHBOR_DISTANCE_THRESHOLD_DEFAULT)
, IsFirstIteration(true)
, T_keyframe_current(Eigen::Isometry3f::Identity())
, T_odometry_current(Eigen::Isometry3f::Identity())
, T_odometry_keyframe(Eigen::Isometry3f::Identity())
, NewPointCloudReceived()
, KeyFramePointCloud()
, KdTreeInstance()
{
    SetupParameters();

    SubscriptionPtr = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::PointCloud_Callback, this, _1));

}

void PointCloudProcessor::PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg)
{
    ExtractPointsFromNewPointCloud(pointCloudMsg);

    if(IsFirstIteration)
    {
        KeyFramePointCloud = NewPointCloudReceived;
        KdTreeInstance.BuildTree(KeyFramePointCloud);
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
    this->declare_parameter<int>("maximumNeighborDistanceThreshold", MAXIMUM_NEIGHBOR_DISTANCE_THRESHOLD_DEFAULT);

    this->get_parameter("minRange", MinRange);
    this->get_parameter("maxRange", MaxRange);
    this->get_parameter("intensityThreshold", IntensityThreshold);
    this->get_parameter("maximumNeighborDistanceThreshold", MaximumNeighborDistanceThreshold);
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
                Eigen::Vector3f newPoint(x, y, z);
                NewPointCloudReceived.push_back(newPoint);
            }
        }
    }
}

void PointCloudProcessor::IterativeClosestPoint(const std::vector<Eigen::Vector3f>& newPointCloudPoints)
{
    constexpr int MAX_ICP_ITERATIONS = 10;
    constexpr float ROTATION_EPSILON = 1e-4;
    constexpr float TRANSLATION_EPSILON = 1e-4;

    Eigen::Matrix3f R = T_keyframe_current.linear();
    Eigen::Vector3f t = T_keyframe_current.translation();

    for(int iter = 0; iter < MAX_ICP_ITERATIONS; ++iter)
    {
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6, 6);
        Eigen::VectorXf b = Eigen::VectorXf::Zero(6);

        for(const auto& point : newPointCloudPoints)
        {
            Eigen::Vector3f pTransformed = R * point + t;

            KdTree::NearestNeighborResult nearestNeighborResult = KdTreeInstance.GetNearestNeighbor(pTransformed);

            if(nearestNeighborResult.distance < MaximumNeighborDistanceThreshold)
            {
                float error = nearestNeighborResult.normal.transpose().dot(pTransformed - nearestNeighborResult.point);
            
                Eigen::Matrix<float, 1, 6> Jacobian;

                Eigen::Matrix3f px = GetSkewMatrix(pTransformed);
                Eigen::Matrix<float, 1, 3> JacobianRotation = -nearestNeighborResult.normal.transpose() * R * px;
                Eigen::Matrix<float, 1, 3> JacobianTranslation = nearestNeighborResult.normal.transpose();
                Jacobian << JacobianRotation, JacobianTranslation;

                H += Jacobian.transpose() * Jacobian;
                b += Jacobian.transpose() * error;
            }
        }

        Eigen::VectorXf dx = H.ldlt().solve(-b);

        Eigen::Vector3f dr = dx.head<3>();
        Eigen::Vector3f dt = dx.tail<3>();

        R = R * ComputeExpSO3(dr);
        t = t + dt;

        if(dr.norm() < ROTATION_EPSILON && dt.norm() < TRANSLATION_EPSILON)
        {
            break;
        }
    }

    T_keyframe_current.linear() = R;
    T_keyframe_current.translation() = t;
}

Eigen::Matrix3f PointCloudProcessor::GetSkewMatrix(const Eigen::Vector3f& point)
{
    Eigen::Matrix3f skewMatrix;

    skewMatrix <<      0.0f, -point.z(),  point.y(),
                  point.z(),       0.0f, -point.x(),
                 -point.y(),  point.x(),       0.0f;

    return skewMatrix;
}

Eigen::Matrix3f PointCloudProcessor::ComputeExpSO3(const Eigen::Vector3f& rotationVector)
{
    constexpr float EPSILON = 1e-6f;
    constexpr float APPROXIMATION_WHEN_ANGLE_IS_SMALL = 0.5f;

    Eigen::Matrix3f validRotationMatrix;
    Eigen::Matrix3f IdentityMatrix = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f skewMatrix = GetSkewMatrix(rotationVector);
    float thetaMagnitude = rotationVector.norm();

    if(thetaMagnitude < EPSILON)
    {
        validRotationMatrix = IdentityMatrix + skewMatrix + (APPROXIMATION_WHEN_ANGLE_IS_SMALL * (skewMatrix * skewMatrix));
    }
    else
    {
        float sinTheta = std::sin(thetaMagnitude);
        float cosTheta = std::cos(thetaMagnitude);

        validRotationMatrix = IdentityMatrix + ((sinTheta / thetaMagnitude) * skewMatrix) + (((1.0f - cosTheta) / (thetaMagnitude * thetaMagnitude)) * (skewMatrix * skewMatrix));
    }

    return validRotationMatrix;
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
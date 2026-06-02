#include "st199010/PointCloudProcessor.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

namespace
{
    constexpr int MIN_RANGE_DEFAULT = 5;
    constexpr int MAX_RANGE_DEFAULT = 100;
    constexpr int INTENSITY_THRESHOLD_DEFAULT = 9000;
    constexpr int MAXIMUM_NEIGHBOR_DISTANCE_THRESHOLD_DEFAULT = 2;
    constexpr int RESET_COUNTER = 0;
}

PointCloudProcessor:: PointCloudProcessor()
: Node("point_cloud_processor_node")
, KeyFrameCounter(RESET_COUNTER)
, KeyFrameStepsUpdateThreshold(10)
, MinRange(MIN_RANGE_DEFAULT)
, MaxRange(MAX_RANGE_DEFAULT)
, IntensityThreshold(INTENSITY_THRESHOLD_DEFAULT)
, MaximumNeighborDistanceThreshold(MAXIMUM_NEIGHBOR_DISTANCE_THRESHOLD_DEFAULT)
, WriteToFile(true)
, IsFirstIteration(true)
, T_keyframe_current(Eigen::Isometry3f::Identity())
, T_odometry_current(Eigen::Isometry3f::Identity())
, T_odometry_keyframe(Eigen::Isometry3f::Identity())
, NewPointCloudReceived()
, KeyFramePointCloud()
, TrajectoryFile()
, KdTreeInstance()
{
    SetupParameters();
    OpenFileToWriteTrajectory();

    SubscriptionPtr = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::PointCloud_Callback, this, _1));
    TfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

PointCloudProcessor::~PointCloudProcessor()
{
    if(TrajectoryFile.is_open())
    {
        TrajectoryFile.close();
    }
}

void PointCloudProcessor::PointCloud_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg)
{
    //RCLCPP_INFO(this->get_logger(), "Recibi nueva nube de puntos. MinRange: %d, MaxRange: %d, IntensityThreshold: %d", MinRange, MaxRange, IntensityThreshold);

    ExtractPointsFromNewPointCloud(pointCloudMsg);  //que regrese un vector de puntos en vez de llenar el atributo de la clase?

    if(IsFirstIteration)
    {
        KeyFramePointCloud = NewPointCloudReceived;
        KdTreeInstance.BuildTree(KeyFramePointCloud);       //seria posible incluso quitar KeyframePointCloud y construir el kdTree directamente con NewPointCloudReceived?
        T_odometry_keyframe = Eigen::Isometry3f::Identity();

        IsFirstIteration = false;
    }
    else
    {
        KeyFrameCounter++;

        RCLCPP_INFO(this->get_logger(), "Voy a meterme a ICP, puntos en newPointCloud: %zu", NewPointCloudReceived.size());

        IterativeClosestPoint(NewPointCloudReceived);   //quitar el parametro?

        //RCLCPP_INFO(this->get_logger(), "Termine ICP");

        T_odometry_current = T_odometry_keyframe * T_keyframe_current;

        //if(KeyFrameCounter == KeyFrameStepsUpdateThreshold)         //cambiar criterio de update de Keyframe?
        if(T_keyframe_current.translation().norm() > 0.4f)
        {
            KeyFramePointCloud = NewPointCloudReceived;
            KdTreeInstance.BuildTree(KeyFramePointCloud);

            T_odometry_keyframe = T_odometry_current;
            T_keyframe_current = Eigen::Isometry3f::Identity();

            KeyFrameCounter = RESET_COUNTER;
        }
    }
    
    PublishTransform(pointCloudMsg->header.stamp, pointCloudMsg->header.frame_id);     //parametros?
    WriteOdometryToFile(pointCloudMsg->header.stamp);
}

void PointCloudProcessor::SetupParameters()
{
    this->declare_parameter<int>("minRange", MIN_RANGE_DEFAULT);
    this->declare_parameter<int>("maxRange", MAX_RANGE_DEFAULT);
    this->declare_parameter<int>("intensityThreshold", INTENSITY_THRESHOLD_DEFAULT);
    this->declare_parameter<int>("maximumNeighborDistanceThreshold", MAXIMUM_NEIGHBOR_DISTANCE_THRESHOLD_DEFAULT);
    this->declare_parameter<bool>("writeToFile", true);

    this->get_parameter("minRange", MinRange);
    this->get_parameter("maxRange", MaxRange);
    this->get_parameter("intensityThreshold", IntensityThreshold);
    this->get_parameter("maximumNeighborDistanceThreshold", MaximumNeighborDistanceThreshold);
    this->get_parameter("writeToFile", WriteToFile);
}

void PointCloudProcessor::OpenFileToWriteTrajectory()
{
    if(WriteToFile)
    {
        TrajectoryFile.open("./estimate.txt");

        if(TrajectoryFile.is_open())
        {
            TrajectoryFile << "# timestamp x y z qx qy qz qw\n";
            RCLCPP_INFO(this->get_logger(), "File opened successfully for writing trajectory.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing trajectory.");
        }
    }
}

void PointCloudProcessor::ExtractPointsFromNewPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg)
{
    constexpr int SUSBSAMPLING_FACTOR_NEW_CLOUD_LIST = 10;

    NewPointCloudReceived.clear();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointCloudMsg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointCloudMsg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointCloudMsg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*pointCloudMsg, "intensity");

    int subsamplePointCounter = RESET_COUNTER;

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
                if(subsamplePointCounter == SUSBSAMPLING_FACTOR_NEW_CLOUD_LIST)
                {
                    Eigen::Vector3f newPoint(x, y, z);
                    NewPointCloudReceived.push_back(newPoint);
                    subsamplePointCounter = RESET_COUNTER;
                }
                else
                {
                    subsamplePointCounter++;
                }
            }
        }
    }
}

void PointCloudProcessor::IterativeClosestPoint(const std::vector<Eigen::Vector3f>& newPointCloudPoints)
{
    constexpr int MAX_ICP_ITERATIONS = 3;
    constexpr float ROTATION_EPSILON = 1e-4;
    constexpr float TRANSLATION_EPSILON = 1e-4;
    constexpr int SUBSAMPLING_STEP = 1;

    Eigen::Matrix3f R = T_keyframe_current.linear();
    Eigen::Vector3f t = T_keyframe_current.translation();

    float previousRootMeanSquareError = std::numeric_limits<float>::max();

    for(int iter = 0; iter < MAX_ICP_ITERATIONS; ++iter)
    {
        Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> b = Eigen::Matrix<float, 6, 1>::Zero();

        int validCorrespondences = 0;
        float totalSquaredError = 0.0f;

        for(size_t i = 0; i < newPointCloudPoints.size(); i += SUBSAMPLING_STEP)
        {
            const auto& point = newPointCloudPoints[i];

            Eigen::Vector3f pTransformed = R * point + t;

            KdTree::NearestNeighborResult nearestNeighborResult = KdTreeInstance.GetNearestNeighbor(pTransformed);

            if(nearestNeighborResult.distanceSq < (MaximumNeighborDistanceThreshold * MaximumNeighborDistanceThreshold))
            {
                validCorrespondences++;

                float error = nearestNeighborResult.normal.transpose().dot(pTransformed - nearestNeighborResult.point);
                totalSquaredError += error * error;

                Eigen::Matrix<float, 1, 6> Jacobian;

                Eigen::Matrix3f px = GetSkewMatrix(point);
                Eigen::Matrix<float, 1, 3> JacobianRotation = -nearestNeighborResult.normal.transpose() * R * px;
                Eigen::Matrix<float, 1, 3> JacobianTranslation = nearestNeighborResult.normal.transpose();
                Jacobian << JacobianRotation, JacobianTranslation;

                H += Jacobian.transpose() * Jacobian;
                b += Jacobian.transpose() * error;
            }
        }

        if(validCorrespondences < 100)
        {
            RCLCPP_WARN(this->get_logger(), "Less than 100 valid correspondences. Breaking ICP iteration.");
            break;
        }

        float rootMeanSquaredError = std::sqrt(totalSquaredError / validCorrespondences);
        RCLCPP_INFO(this->get_logger(), "ICP Iteration %d: Valid Correspondences: %d, RMSE: %f", iter + 1, validCorrespondences, rootMeanSquaredError);

        if(rootMeanSquaredError > previousRootMeanSquareError)
        {
            RCLCPP_WARN(this->get_logger(), "RMSE increased. Breaking ICP iteration.");
            break;
        }

        previousRootMeanSquareError = rootMeanSquaredError;

        Eigen::Matrix<float, 6, 1> dx = H.ldlt().solve(-b);

        if(!dx.allFinite())
        {
            RCLCPP_WARN(this->get_logger(), "Non-finite values in ICP solution, stopping iteration.");
            break;
        }
        else
        {
            Eigen::Vector3f dr = dx.head<3>();
            Eigen::Vector3f dt = dx.tail<3>();

            RCLCPP_INFO(this->get_logger(), "dr_norm: %0.6f, dt_norm: %0.6f", dr.norm(), dt.norm());

            //if(dr.norm() > 0.08f || dt.norm() > 0.15f)
            //{
            //    RCLCPP_WARN(this->get_logger(), "Large transformation update detected. Breaking ICP iteration.");
            //    break;
            //}


            R = R * ComputeExpSO3(dr);
            t = t + dt;

            if(dr.norm() < ROTATION_EPSILON && dt.norm() < TRANSLATION_EPSILON)
            {
                break;
            }
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

void PointCloudProcessor::PublishTransform(const rclcpp::Time& timestamp, const std::string& frame_id)   //parametros?
{
    RCLCPP_INFO(this->get_logger(), "Voy a publicar translation x: %0.4f, y: %0.4f, z: %0.4f", T_odometry_current.translation().x(), T_odometry_current.translation().y(), T_odometry_current.translation().z());

    Eigen::Quaternionf quaternion(T_odometry_current.linear());
    quaternion.normalize();

    geometry_msgs::msg::TransformStamped transformMsg;

    transformMsg.header.stamp = timestamp;
    transformMsg.header.frame_id = "odom_frame";
    transformMsg.child_frame_id = frame_id;

    transformMsg.transform.translation.x = T_odometry_current.translation().x();
    transformMsg.transform.translation.y = T_odometry_current.translation().y();
    transformMsg.transform.translation.z = T_odometry_current.translation().z();

    transformMsg.transform.rotation.x = quaternion.x();
    transformMsg.transform.rotation.y = quaternion.y();
    transformMsg.transform.rotation.z = quaternion.z();
    transformMsg.transform.rotation.w = quaternion.w();

    TfBroadcaster->sendTransform(transformMsg);
}

void PointCloudProcessor::WriteOdometryToFile(const rclcpp::Time& timestamp)
{
    if(WriteToFile && TrajectoryFile.is_open())
    {
        Eigen::Quaternionf quaternion(T_odometry_current.linear());
        quaternion.normalize();

        TrajectoryFile << std::fixed << std::setprecision(6) << timestamp.seconds() << " "
                       << T_odometry_current.translation().x() << " "
                       << T_odometry_current.translation().y() << " "
                       << T_odometry_current.translation().z() << " "
                       << quaternion.x() << " "
                       << quaternion.y() << " "
                       << quaternion.z() << " "
                       << quaternion.w() << "\n";
    }
}

/*
    Cosas que checar:
        -Comparar contra el ground truth 
        -Hacer el test de nube vs nube iguales = identidad

*/
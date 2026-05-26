#include "3d-lidar-odometry/KdTree.hpp"
#include <Eigen/Dense>

void KdTree::BuildTree(const std::vector<Eigen::Vector3f>& pointsVector)
{
    Root = BuildTreeRecursive(pointsVector);
}

std::unique_ptr<KdTree::Node> KdTree::BuildTreeRecursive(const std::vector<Eigen::Vector3f>& pointsVector)
{
    constexpr int PRINCIPAL_EIGENVECTOR_COLUMN_INDEX = 2;
    constexpr int NORMAL_EIGENVECTOR_COLUMN_INDEX = 0;
    constexpr int MIN_POINTS_THRESHOLD = 10;
    constexpr int ZERO_DISTANCE = 0;

    auto nodePtr = std::make_unique<KdTree::Node>();

    Eigen::Vector3f mean = ComputeMean(pointsVector);
    Eigen::Matrix3f covariance = ComputeCovariance(pointsVector, mean);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    Eigen::Vector3f principal_eigenvector = eigen_solver.eigenvectors().col(PRINCIPAL_EIGENVECTOR_COLUMN_INDEX);
    
    nodePtr->mean = mean;
    nodePtr->splitDirection = principal_eigenvector;

    if(pointsVector.size() <= MIN_POINTS_THRESHOLD)
    {
        nodePtr->isLeaf = true;
        nodePtr->left = nullptr;
        nodePtr->right = nullptr;
        nodePtr->normalVector = eigen_solver.eigenvectors().col(NORMAL_EIGENVECTOR_COLUMN_INDEX);
        nodePtr->points = pointsVector;
    }
    else
    {
        std::vector<Eigen::Vector3f> left_points;
        std::vector<Eigen::Vector3f> right_points;

        for (const auto& point : pointsVector)
        {
            if (principal_eigenvector.dot(point - mean) > ZERO_DISTANCE)
                right_points.push_back(point);
            else
                left_points.push_back(point);
        }
        
        if(left_points.empty() || right_points.empty())
        {
            nodePtr->isLeaf = true;
            nodePtr->left = nullptr;
            nodePtr->right = nullptr;
            nodePtr->normalVector = eigen_solver.eigenvectors().col(NORMAL_EIGENVECTOR_COLUMN_INDEX);
            nodePtr->points = pointsVector;
        }
        else
        {
            nodePtr->isLeaf = false;
            nodePtr->left = BuildTreeRecursive(left_points);
            nodePtr->right = BuildTreeRecursive(right_points);
        }
    }

    return nodePtr;
}

Eigen::Vector3f KdTree::ComputeMean(const std::vector<Eigen::Vector3f>& points)
{
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();

    for (const auto& point : points)
        sum += point;

    return sum / points.size();
}

Eigen::Matrix3f KdTree::ComputeCovariance(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& mean)
{
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
 
    for (const auto& point : points)
    {
        Eigen::Vector3f centered = point - mean;
        covariance += centered * centered.transpose();
     }

    return covariance / points.size();
}
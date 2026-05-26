#include "3d-lidar-odometry/KdTree.hpp"
#include <Eigen/Dense>

void KdTree::buildTree(const std::vector<Eigen::Vector3f>& pointsVector)
{
    root = buildTreeRecursive(pointsVector);
}

std::unique_ptr<KdTree::Node> KdTree::buildTreeRecursive(const std::vector<Eigen::Vector3f>& pointsVector)
{
    constexpr int PRINCIPAL_EIGENVECTOR_COLUMN_INDEX = 2;
    constexpr int NORMAL_EIGENVECTOR_COLUMN_INDEX = 0;
    constexpr int MIN_POINTS_THRESHOLD = 10;
    constexpr int ZERO_DISTANCE = 0;

    KdTree::Node node;

    Eigen::Vector3f mean = ComputeMean(pointsVector);
    Eigen::Matrix3f covariance = ComputeCovariance(pointsVector, mean);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    Eigen::Vector3f principal_eigenvector = eigen_solver.eigenvectors().col(PRINCIPAL_EIGENVECTOR_COLUMN_INDEX);
    
    node.mean = mean;
    node.splitDirection = principal_eigenvector;

    if(pointsVector.size() <= MIN_POINTS_THRESHOLD)
    {
        node.isLeaf = true;
        node.left = nullptr;
        node.right = nullptr;
        node.normalVector = eigen_solver.eigenvectors().col(NORMAL_EIGENVECTOR_COLUMN_INDEX);
        node.points = pointsVector;
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
            node.isLeaf = true;
            node.left = nullptr;
            node.right = nullptr;
            node.normalVector = eigen_solver.eigenvectors().col(NORMAL_EIGENVECTOR_COLUMN_INDEX);
            node.points = pointsVector;
        }
        else
        {
            node.isLeaf = false;
            node.left = buildTreeRecursive(left_points);
            node.right = buildTreeRecursive(right_points);
        }
    }

    std::unique_ptr<KdTree::Node> nodePtr = std::make_unique<KdTree::Node>(node);

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
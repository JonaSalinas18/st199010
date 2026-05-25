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

    if(pointsVector.size() < MIN_POINTS_THRESHOLD)
    {
        node.isLeaf = true;
        //guardar points?
        //return
    }

    Eigen::Vector3f mean = ComputeMean(pointsVector);
    Eigen::Matrix3f covariance = ComputeCovariance(pointsVector, mean);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    Eigen::Vector3f principal_eigenvector = eigen_solver.eigenvectors().col(PRINCIPAL_EIGENVECTOR_COLUMN_INDEX);
    Eigen::Vector3f normal_vector = eigen_solver.eigenvectors().col(NORMAL_EIGENVECTOR_COLUMN_INDEX);

    std::vector<Eigen::Vector3f> left_points;
    std::vector<Eigen::Vector3f> right_points;

    for (const auto& point : pointsVector)
    {
        if (principal_eigenvector.dot(point - mean) > ZERO_DISTANCE)
            right_points.push_back(point);
        else
            left_points.push_back(point);
    }
 
    node.mean = mean;
    node.splitDirection = principal_eigenvector;
    node.normalVector = normal_vector;
    node.left = buildTreeRecursive(left_points);
    node.right = buildTreeRecursive(right_points);

    std::unique_ptr<KdTree::Node> node_ptr = std::make_unique<KdTree::Node>(node);

    return node_ptr;
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
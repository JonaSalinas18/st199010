#include "st199010/KdTree.hpp"
#include <Eigen/Dense>

namespace
{
    constexpr float ZERO_DISTANCE = 0;
}

void KdTree::BuildTree(const std::vector<Eigen::Vector3f>& pointsVector)
{
    Root = BuildTreeRecursive(pointsVector);
}

std::unique_ptr<KdTree::Node> KdTree::BuildTreeRecursive(const std::vector<Eigen::Vector3f>& pointsVector)
{
    constexpr int PRINCIPAL_EIGENVECTOR_COLUMN_INDEX = 2;
    constexpr int NORMAL_EIGENVECTOR_COLUMN_INDEX = 0;
    constexpr int MIN_POINTS_THRESHOLD = 10;

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

KdTree::NearestNeighborResult KdTree::GetNearestNeighbor(const Eigen::Vector3f& queryPoint)
{
    NearestNeighborResult bestNeighbor;
    bestNeighbor.found = false;
    bestNeighbor.distance = std::numeric_limits<float>::max();

    GetNearestNeighborRecursive(Root, queryPoint, bestNeighbor);

    return bestNeighbor;
}

void KdTree::GetNearestNeighborRecursive(const std::unique_ptr<Node>& node, const Eigen::Vector3f& queryPoint, NearestNeighborResult& bestResult)
{
    if(node->isLeaf)
    {
        for (const auto& point : node->points)
        {
            float pointDistance = (queryPoint - point).norm();
            if (pointDistance < bestResult.distance)
            {
                bestResult.point = point;
                bestResult.normal = node->normalVector;
                bestResult.distance = pointDistance;
                bestResult.found = true;        //es util?
            }
        }
    }
    else
    {
        float splitDistance = node->splitDirection.dot(queryPoint - node->mean);
        const std::unique_ptr<Node>& firstSearch = splitDistance > ZERO_DISTANCE ? node->right : node->left;
        const std::unique_ptr<Node>& secondSearch = splitDistance > ZERO_DISTANCE ? node->left : node->right;

        GetNearestNeighborRecursive(firstSearch, queryPoint, bestResult);

        if (std::abs(splitDistance) < bestResult.distance)
        {
            GetNearestNeighborRecursive(secondSearch, queryPoint, bestResult);
        }
    }
}
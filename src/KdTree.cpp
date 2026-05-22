#include "3d-lidar-odometry/KdTree.hpp"

void KdTree::buildTree(const std::vector<Eigen::Vector3f>& pointsVector)
{
    if(pointsVector.size() < 10)
    {
        //node->isLeaf = true;
        //return
    }

    Eigen::Vector3f mean = ComputeMean(pointsVector);
    Eigen::Matrix3f covariance = ComputeCovariance(pointsVector, mean);


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
    //Revisar logica:
    /* 
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : points)
    {
        Eigen::Vector3f centered = point - mean;
        covariance += centered * centered.transpose();
     }
    return covariance / points.size();
    */
    return Eigen::Matrix3f::Zero();
}
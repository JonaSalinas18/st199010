#pragma once

#include <Eigen/Geometry>
#include <memory>

class KdTree
{
    public:
        KdTree() = default;
        
        void BuildTree(const std::vector<Eigen::Vector3f>& pointsVector);
        //std::unique_ptr<Node> GetNearestNeighbor(const Eigen::Vector3f& queryPoint);
        //GetNearestNeighbor

    private:
        struct Node
        {
            Eigen::Vector3f mean;
            Eigen::Vector3f splitDirection;
            Eigen::Vector3f normalVector;
            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            std::vector<Eigen::Vector3f> points;
            bool isLeaf;
        };

        std::unique_ptr<Node> Root;

        Eigen::Vector3f ComputeMean(const std::vector<Eigen::Vector3f>& points);
        Eigen::Matrix3f ComputeCovariance(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& mean);
        std::unique_ptr<Node> BuildTreeRecursive(const std::vector<Eigen::Vector3f>& pointsVector);
};
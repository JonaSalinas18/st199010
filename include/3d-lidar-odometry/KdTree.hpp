#pragma once

#include <Eigen/Geometry>
#include <memory>

class KdTree
{
    public:
        KdTree() = default;
        
        void buildTree(const std::vector<Eigen::Vector3f>& pointsVector);
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
            bool isLeaf;

            //Node(const Eigen::Vector3d& pt, int idx) : point(pt), index(idx), left(nullptr), right(nullptr) {}
        };

        std::unique_ptr<Node> root;

        Eigen::Vector3f ComputeMean(const std::vector<Eigen::Vector3f>& points);
        Eigen::Matrix3f ComputeCovariance(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& mean);
        std::unique_ptr<Node> buildTreeRecursive(const std::vector<Eigen::Vector3f>& pointsVector);
};


class KdTree
{
    public:
        KdTree() = default;
        
        void buildTree(const std::vector<Eigen::Vector3d>& pointsVector);
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

            Node(const Eigen::Vector3d& pt, int idx) : point(pt), index(idx), left(nullptr), right(nullptr) {}
        };

        std::unique_ptr<Node> root;

        float ComputeMean(const std::vector<Eigen::Vector3d>& points);
        //Eigen::Matrix3f ComputeCovariance(const std::vector<Eigen::Vector3d);
}
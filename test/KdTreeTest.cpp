#include <gtest/gtest.h>
#include "st199010/KdTree.hpp"

std::vector<Eigen::Vector3f> samplePointCloud = {
    {12.3f, 47.1f,  8.6f},
    {35.0f,  3.4f, 22.9f},
    {18.7f, 29.5f, 41.2f},
    { 6.1f, 14.8f, 33.7f},
    {44.5f, 50.0f,  1.3f},
    {27.9f,  9.2f, 16.4f},
    { 0.5f, 38.6f, 25.0f},
    {49.3f, 21.7f,  7.8f},
    {31.4f,  5.9f, 43.1f},
    {15.6f, 42.3f, 19.0f},
    { 8.2f, 17.5f, 36.4f},
    {23.0f, 48.9f,  2.7f},
    {40.8f, 11.3f, 28.5f},
    { 3.7f, 26.4f, 45.9f},
    {46.1f, 33.0f, 12.6f},
    {19.4f,  0.8f, 37.2f},
    {11.0f, 44.7f, 24.3f},
    {38.5f, 16.2f,  5.1f},
    {25.7f,  7.6f, 49.8f},
    { 2.9f, 39.1f, 31.5f},
    {47.6f, 23.4f, 14.7f},
    {33.2f, 50.0f,  0.3f},
    { 9.8f, 12.0f, 42.6f},
    {21.5f, 45.3f, 27.9f},
    {16.0f, 28.7f,  9.4f},
    {42.3f,  4.5f, 35.8f},
    { 5.4f, 36.9f, 18.1f},
    {29.6f, 19.8f, 46.5f},
    {13.1f, 41.2f,  3.6f},
    {48.0f,  8.3f, 23.7f},
    {37.4f, 24.6f, 11.0f},
    { 1.8f, 15.9f, 40.4f},
    {26.3f, 49.4f, 32.2f},
    {43.7f,  2.1f, 17.5f},
    {10.5f, 31.8f, 44.9f},
    {34.9f, 46.0f,  6.3f},
    {22.6f, 13.4f, 29.1f},
    { 7.3f, 40.7f, 50.0f},
    {50.0f, 18.5f, 21.8f},
    {14.2f,  6.7f, 38.3f},
    {39.1f, 35.2f,  4.0f},
    {17.8f, 22.9f, 47.3f},
    { 4.6f, 43.5f, 15.2f},
    {28.0f,  1.4f, 33.6f},
    {45.3f, 30.1f,  8.9f},
    {20.9f, 10.6f, 26.4f},
    {32.5f, 47.8f, 13.7f},
    { 6.8f, 24.0f, 41.0f},
    {41.6f,  37.3f, 2.5f},
    {24.1f, 9.0f,  48.7f}
};

TEST(KdTreeTest, GivenSampleKdTree_WhenQueryingEachPoint_ThenNearestNeighborIsTheSamePoint)
{
    constexpr float DISTANCE_ZERO = 0.0f;

    KdTree treeTest;

    treeTest.BuildTree(samplePointCloud);

    for (const auto& point : samplePointCloud)
    {
        KdTree::NearestNeighborResult resultNeighbor = treeTest.GetNearestNeighbor(point);

        ASSERT_TRUE(resultNeighbor.found);
        EXPECT_FLOAT_EQ(resultNeighbor.point.x(), point.x());
        EXPECT_FLOAT_EQ(resultNeighbor.point.y(), point.y());
        EXPECT_FLOAT_EQ(resultNeighbor.point.z(), point.z());
        EXPECT_FLOAT_EQ(resultNeighbor.distanceSq, DISTANCE_ZERO);
    }
}

TEST(KdTreeTest, GivenKdTreeNearestNeighborQuery_ThenResultIsTheSameAsBruteForceResult)
{
    KdTree treeTest;
    treeTest.BuildTree(samplePointCloud);

    Eigen::Vector3f queryPoint(12.3f, 47.1f, 8.6f);

    KdTree::NearestNeighborResult resultNeighbor = treeTest.GetNearestNeighbor(queryPoint);

    Eigen::Vector3f bruteForceNearestNeighbor;
    float shortestDistance = std::numeric_limits<float>::max();

    for(const auto& point : samplePointCloud)
    {
        float euclideanDistanceSq = (queryPoint - point).squaredNorm();

        if (euclideanDistanceSq < shortestDistance)
        {
            shortestDistance = euclideanDistanceSq;
            bruteForceNearestNeighbor = point;
        }
    }

    ASSERT_TRUE(resultNeighbor.found);
    EXPECT_FLOAT_EQ(resultNeighbor.point.x(), bruteForceNearestNeighbor.x());
    EXPECT_FLOAT_EQ(resultNeighbor.point.y(), bruteForceNearestNeighbor.y());
    EXPECT_FLOAT_EQ(resultNeighbor.point.z(), bruteForceNearestNeighbor.z());
    EXPECT_FLOAT_EQ(resultNeighbor.distanceSq, shortestDistance);
}

TEST(KdTreeTest, GivenSampleKdTreeWithTwoGroups_WhenQueryingNearestNeighbor_ThenNormalsMatchesExpected)
{
    std::vector<Eigen::Vector3f> pointsGroupA = {
        {0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f},
        {0.0f, 0.1f, 0.0f}, {0.0f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.1f},
        {0.1f, 0.1f, 0.0f}, {0.2f, 0.1f, 0.0f}, {0.1f, 0.2f, 0.0f},
        {0.2f, 0.2f, 0.0f}
    };

    std::vector<Eigen::Vector3f> pointsGroupB = {
        {100.0f, 0.0f, 0.0f}, {100.1f, 0.0f, 0.0f}, {100.2f, 0.0f, 0.0f},
        {100.0f, 0.1f, 0.0f}, {100.0f, 0.2f, 0.0f}, {100.0f, 0.0f, 0.1f},
        {100.1f, 0.1f, 0.0f}, {100.2f, 0.1f, 0.0f}, {100.1f, 0.2f, 0.0f},
        {100.2f, 0.2f, 0.0f}
    };

    std::vector<Eigen::Vector3f> allPoints;
    allPoints.insert(allPoints.end(), pointsGroupA.begin(), pointsGroupA.end());
    allPoints.insert(allPoints.end(), pointsGroupB.begin(), pointsGroupB.end());

    KdTree kdTreeTest;
    kdTreeTest.BuildTree(allPoints);

    Eigen::Vector3f queryPoint = {0.05f, 0.05f, 0.0f};
    KdTree::NearestNeighborResult resultNeighbor = kdTreeTest.GetNearestNeighbor(queryPoint);

    KdTree referenceKdTreeA;
    referenceKdTreeA.BuildTree(pointsGroupA);
    KdTree::NearestNeighborResult referenceResultA = referenceKdTreeA.GetNearestNeighbor(queryPoint);

    ASSERT_TRUE(resultNeighbor.found);
    EXPECT_FLOAT_EQ(resultNeighbor.normal.x(), referenceResultA.normal.x());
    EXPECT_FLOAT_EQ(resultNeighbor.normal.y(), referenceResultA.normal.y());
    EXPECT_FLOAT_EQ(resultNeighbor.normal.z(), referenceResultA.normal.z());

    queryPoint = {100.05f, 0.05f, 0.0f};
    resultNeighbor = kdTreeTest.GetNearestNeighbor(queryPoint);

    KdTree referenceKdTreeB;
    referenceKdTreeB.BuildTree(pointsGroupB);
    KdTree::NearestNeighborResult referenceResultB = referenceKdTreeB.GetNearestNeighbor(queryPoint);

    ASSERT_TRUE(resultNeighbor.found);
    EXPECT_FLOAT_EQ(resultNeighbor.normal.x(), referenceResultB.normal.x());
    EXPECT_FLOAT_EQ(resultNeighbor.normal.y(), referenceResultB.normal.y());
    EXPECT_FLOAT_EQ(resultNeighbor.normal.z(), referenceResultB.normal.z());
}


/*
Otros tests:
    - Test de normales

    - Teniendo ciertos puntos, buscar el nearest neighbor y testear la normal

*/
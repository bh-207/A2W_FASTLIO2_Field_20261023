#include "pcd2pgm/parallel_filters.hpp"
#include <gtest/gtest.h>

using pcd2pgm::Cloud;

TEST(ParallelGround, FlatGroundAndTallObstacleKeepOriginalOrder)
{
  auto cloud = pcl::make_shared<Cloud>();
  pcl::Indices expected;
  // More than 512 points: schedule(dynamic,128) must distribute real work.
  for (int x = -16; x <= 16; ++x) {
    for (int y = -16; y <= 16; ++y) {
      expected.push_back(static_cast<int>(cloud->size()));
      cloud->push_back(pcl::PointXYZ(x * 0.5f, y * 0.5f, 0.0f));
    }
  }
  cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 2.0f));
  const pcd2pgm::GroundOptions options{32, 1.0f, 0.2f, 2.5f};
  for (int threads : {1, 4}) {
    std::vector<float> windows;
    EXPECT_EQ(pcd2pgm::extractGroundParallel(cloud, options, threads,
      [&](float window, std::size_t, double) { windows.push_back(window); }), expected);
    EXPECT_EQ(windows, (std::vector<float>{3, 5, 9, 17, 33}));
  }
  EXPECT_FLOAT_EQ(cloud->back().z, 2.0f);
}

TEST(ParallelGround, StrictThresholdAndWindowEdges)
{
  auto cloud = pcl::make_shared<Cloud>();
  cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  cloud->push_back(pcl::PointXYZ(1.5f, 0.0f, 0.2f));
  cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.199f));
  for (int threads : {1, 4}) {
    EXPECT_EQ(pcd2pgm::extractGroundParallel(cloud, {3, 1.0f, 0.2f, 2.5f}, threads),
      (pcl::Indices{0, 2}));
  }
}

TEST(ParallelGround, EmptyAndSinglePoint)
{
  auto cloud = pcl::make_shared<Cloud>();
  EXPECT_TRUE(pcd2pgm::extractGroundParallel(cloud, {}, 4).empty());
  cloud->push_back(pcl::PointXYZ(7, -3, 2));
  EXPECT_EQ(pcd2pgm::extractGroundParallel(cloud, {}, 4), (pcl::Indices{0}));
}

TEST(ParallelObstacles, InclusiveHeightsAndStableOrderWithFewNeighbors)
{
  auto ground = pcl::make_shared<Cloud>();
  ground->push_back(pcl::PointXYZ(0, 0, 0));
  auto cloud = pcl::make_shared<Cloud>();
  for (int i = 0; i < 300; ++i) {
    for (float z : {0.125f, 0.124f, 1.5f, 1.501f, -0.1f}) {
      cloud->push_back(pcl::PointXYZ(i, -1, z));
    }
  }
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (int threads : {1, 4}) {
      auto result = pcd2pgm::extractObstaclesParallel(cloud, ground, 0.125, 1.5, threads);
      ASSERT_EQ(result->size(), 600u);
      for (int i = 0; i < 300; ++i) {
        EXPECT_FLOAT_EQ(result->at(2 * i).z, 0.125f);
        EXPECT_FLOAT_EQ(result->at(2 * i + 1).z, 1.5f);
        EXPECT_FLOAT_EQ(result->at(2 * i).x, i);
        EXPECT_FLOAT_EQ(result->at(2 * i).y, -1);
      }
    }
  }
}

TEST(ParallelObstacles, EmptyInputOrGround)
{
  auto empty = pcl::make_shared<Cloud>();
  auto cloud = pcl::make_shared<Cloud>();
  cloud->push_back(pcl::PointXYZ(0, 0, 1));
  EXPECT_TRUE(pcd2pgm::extractObstaclesParallel(cloud, empty, 0.1, 1.5, 4)->empty());
  EXPECT_TRUE(pcd2pgm::extractObstaclesParallel(empty, cloud, 0.1, 1.5, 4)->empty());
}

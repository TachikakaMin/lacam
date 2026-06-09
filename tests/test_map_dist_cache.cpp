#include <filesystem>

#include <lacam.hpp>

#include "gtest/gtest.h"

namespace
{
std::filesystem::path temp_cache_path(const std::string& name)
{
  return std::filesystem::temp_directory_path() / name;
}
}  // namespace

TEST(map_dist_cache, computes_reachable_and_unreachable_distances)
{
  const auto map_path = std::filesystem::path("./tests/assets/disconnected-3x1.map");
  const auto graph = Graph(map_path.string());
  const auto cache =
      build_map_distance_cache(graph, map_path.filename().string(), hash_file(map_path));

  ASSERT_EQ(cache.metadata.width, 3);
  ASSERT_EQ(cache.metadata.height, 1);
  ASSERT_EQ(cache.metadata.traversable_count, 2);
  ASSERT_EQ(cache.get(graph.U[0], graph.U[0]), 0);
  ASSERT_EQ(cache.get(graph.U[0], graph.U[2]), kMapDistanceInf);
}

TEST(map_dist_cache, saves_loads_and_rejects_metadata_mismatch)
{
  const auto map_path = std::filesystem::path("./tests/assets/2x1.map");
  const auto cache_path = temp_cache_path("lacam_map_dist_cache_test.bin");
  std::filesystem::remove(cache_path);

  auto cache = load_or_build_map_distance_cache(map_path, cache_path);
  ASSERT_TRUE(std::filesystem::exists(cache_path));
  ASSERT_EQ(cache.get_by_vertex_id(0, 1), 1);

  auto expected = cache.metadata;
  auto loaded = MapDistanceCache();
  ASSERT_TRUE(load_map_distance_cache(cache_path, expected, loaded));
  ASSERT_EQ(loaded.get_by_vertex_id(1, 0), 1);

  expected.map_hash += 1;
  auto rejected = MapDistanceCache();
  ASSERT_FALSE(load_map_distance_cache(cache_path, expected, rejected));

  std::filesystem::remove(cache_path);
}

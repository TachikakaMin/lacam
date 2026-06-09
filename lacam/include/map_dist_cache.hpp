/*
 * All-pairs shortest-path cache for traversable map cells.
 */
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "graph.hpp"

constexpr int kMapDistanceInf = 100000000;

struct MapDistanceCacheMetadata {
  std::string map_name;
  std::uint64_t map_hash = 0;
  int width = 0;
  int height = 0;
  int traversable_count = 0;
};

struct MapDistanceCache {
  MapDistanceCacheMetadata metadata;
  std::vector<std::vector<int> > distances;

  int get(Vertex* from, Vertex* to) const;
  int get_by_vertex_id(int from_id, int to_id) const;
};

std::uint64_t hash_file(const std::filesystem::path& path);
std::uint64_t estimate_distance_cache_bytes(int traversable_count);
bool load_map_distance_cache(const std::filesystem::path& cache_path,
                             const MapDistanceCacheMetadata& expected,
                             MapDistanceCache& cache);
void save_map_distance_cache(const std::filesystem::path& cache_path,
                             const MapDistanceCache& cache);
MapDistanceCache build_map_distance_cache(const Graph& graph,
                                          const std::string& map_name,
                                          std::uint64_t map_hash);
MapDistanceCache load_or_build_map_distance_cache(
    const std::filesystem::path& map_path,
    const std::filesystem::path& cache_path, int verbose = 0);

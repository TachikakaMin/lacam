#include "../include/map_dist_cache.hpp"

#include <algorithm>
#include <atomic>
#include <deque>
#include <stdexcept>
#include <thread>

#include "../include/utils.hpp"

namespace
{
  constexpr char kMagic[8] = {'M', 'A', 'P', 'D', 'I', 'S', 'T', '1'};
  constexpr std::uint32_t kVersion = 1;

  void write_u32(std::ofstream& out, std::uint32_t value)
  {
    out.write(reinterpret_cast<const char*>(&value), sizeof(value));
  }

  void write_u64(std::ofstream& out, std::uint64_t value)
  {
    out.write(reinterpret_cast<const char*>(&value), sizeof(value));
  }

  std::uint32_t read_u32(std::ifstream& in)
  {
    auto value = std::uint32_t();
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
  }

  std::uint64_t read_u64(std::ifstream& in)
  {
    auto value = std::uint64_t();
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
  }

  std::string read_string(std::ifstream& in)
  {
    const auto size = read_u32(in);
    auto value = std::string(size, '\0');
    in.read(value.data(), size);
    return value;
  }

  void write_string(std::ofstream& out, const std::string& value)
  {
    write_u32(out, static_cast<std::uint32_t>(value.size()));
    out.write(value.data(), value.size());
  }

  bool metadata_matches(const MapDistanceCacheMetadata& lhs,
                        const MapDistanceCacheMetadata& rhs)
  {
    return lhs.map_name == rhs.map_name && lhs.map_hash == rhs.map_hash &&
           lhs.width == rhs.width && lhs.height == rhs.height &&
           lhs.traversable_count == rhs.traversable_count;
  }

  std::vector<int> bfs_distances_from(const Graph& graph, Vertex* source)
  {
    auto dist = std::vector<int>(graph.size(), kMapDistanceInf);
    auto open = std::deque<Vertex*>();
    dist[source->id] = 0;
    open.push_back(source);

    while (!open.empty()) {
      auto v = open.front();
      open.pop_front();
      for (auto n : v->neighbor) {
        if (dist[n->id] <= dist[v->id] + 1) continue;
        dist[n->id] = dist[v->id] + 1;
        open.push_back(n);
      }
    }
    return dist;
  }
}  // namespace

int MapDistanceCache::get(Vertex* from, Vertex* to) const
{
  if (from == nullptr || to == nullptr) return kMapDistanceInf;
  return get_by_vertex_id(from->id, to->id);
}

int MapDistanceCache::get_by_vertex_id(int from_id, int to_id) const
{
  if (from_id < 0 || from_id >= static_cast<int>(distances.size())) {
    return kMapDistanceInf;
  }
  if (to_id < 0 || to_id >= static_cast<int>(distances[from_id].size())) {
    return kMapDistanceInf;
  }
  return distances[from_id][to_id];
}

int MapDistanceRows::get(Vertex* from, Vertex* goal) const
{
  if (from == nullptr || goal == nullptr) return kMapDistanceInf;
  const auto iter = row_by_vertex_id.find(goal->id);
  if (iter == row_by_vertex_id.end() || iter->second >= rows.size() ||
      from->id < 0 || from->id >= static_cast<int>(rows[iter->second].size())) {
    return kMapDistanceInf;
  }
  return rows[iter->second][from->id];
}

std::uint64_t hash_file(const std::filesystem::path& path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("failed to open map for hashing: " +
                             path.string());

  auto hash = 1469598103934665603ULL;
  char c;
  while (in.get(c)) {
    hash ^= static_cast<unsigned char>(c);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t estimate_distance_cache_bytes(int traversable_count)
{
  const auto n = static_cast<std::uint64_t>(traversable_count);
  return n * n * sizeof(int);
}

bool load_map_distance_cache(const std::filesystem::path& cache_path,
                             const MapDistanceCacheMetadata& expected,
                             MapDistanceCache& cache)
{
  std::ifstream in(cache_path, std::ios::binary);
  if (!in) return false;

  char magic[8];
  in.read(magic, sizeof(magic));
  if (!in ||
      !std::equal(std::begin(magic), std::end(magic), std::begin(kMagic))) {
    return false;
  }
  if (read_u32(in) != kVersion) return false;

  auto metadata = MapDistanceCacheMetadata();
  metadata.map_name = read_string(in);
  metadata.map_hash = read_u64(in);
  metadata.width = static_cast<int>(read_u32(in));
  metadata.height = static_cast<int>(read_u32(in));
  metadata.traversable_count = static_cast<int>(read_u32(in));
  if (!metadata_matches(metadata, expected)) return false;

  auto distances = std::vector<std::vector<int> >(
      metadata.traversable_count,
      std::vector<int>(metadata.traversable_count, kMapDistanceInf));
  for (auto& row : distances) {
    in.read(reinterpret_cast<char*>(row.data()),
            static_cast<std::streamsize>(row.size() * sizeof(int)));
    if (!in) return false;
  }

  cache.metadata = metadata;
  cache.distances = std::move(distances);
  return true;
}

bool load_map_distance_rows(const std::filesystem::path& cache_path,
                            const MapDistanceCacheMetadata& expected,
                            const std::vector<int>& source_vertex_ids,
                            MapDistanceRows& cache)
{
  std::ifstream in(cache_path, std::ios::binary);
  if (!in) return false;

  char magic[8];
  in.read(magic, sizeof(magic));
  if (!in ||
      !std::equal(std::begin(magic), std::end(magic), std::begin(kMagic)) ||
      read_u32(in) != kVersion) {
    return false;
  }
  auto metadata = MapDistanceCacheMetadata();
  metadata.map_name = read_string(in);
  metadata.map_hash = read_u64(in);
  metadata.width = static_cast<int>(read_u32(in));
  metadata.height = static_cast<int>(read_u32(in));
  metadata.traversable_count = static_cast<int>(read_u32(in));
  if (!metadata_matches(metadata, expected)) return false;
  const auto data_offset = in.tellg();

  auto unique_sources = source_vertex_ids;
  std::sort(unique_sources.begin(), unique_sources.end());
  unique_sources.erase(
      std::unique(unique_sources.begin(), unique_sources.end()),
      unique_sources.end());
  auto loaded = MapDistanceRows();
  loaded.metadata = metadata;
  loaded.rows.reserve(unique_sources.size());
  for (const auto source : unique_sources) {
    if (source < 0 || source >= metadata.traversable_count) return false;
    const auto row_offset =
        data_offset +
        static_cast<std::streamoff>(source) *
            static_cast<std::streamoff>(metadata.traversable_count) *
            static_cast<std::streamoff>(sizeof(int));
    in.seekg(row_offset);
    auto row = std::vector<int>(metadata.traversable_count, kMapDistanceInf);
    in.read(reinterpret_cast<char*>(row.data()),
            static_cast<std::streamsize>(row.size() * sizeof(int)));
    if (!in) return false;
    loaded.row_by_vertex_id[source] = loaded.rows.size();
    loaded.rows.push_back(std::move(row));
  }
  cache = std::move(loaded);
  return true;
}

void save_map_distance_cache(const std::filesystem::path& cache_path,
                             const MapDistanceCache& cache)
{
  if (cache_path.has_parent_path()) {
    std::filesystem::create_directories(cache_path.parent_path());
  }

  std::ofstream out(cache_path, std::ios::binary);
  if (!out)
    throw std::runtime_error("failed to write distance cache: " +
                             cache_path.string());

  out.write(kMagic, sizeof(kMagic));
  write_u32(out, kVersion);
  write_string(out, cache.metadata.map_name);
  write_u64(out, cache.metadata.map_hash);
  write_u32(out, static_cast<std::uint32_t>(cache.metadata.width));
  write_u32(out, static_cast<std::uint32_t>(cache.metadata.height));
  write_u32(out, static_cast<std::uint32_t>(cache.metadata.traversable_count));
  for (const auto& row : cache.distances) {
    out.write(reinterpret_cast<const char*>(row.data()),
              static_cast<std::streamsize>(row.size() * sizeof(int)));
  }
}

MapDistanceCache build_map_distance_cache(const Graph& graph,
                                          const std::string& map_name,
                                          std::uint64_t map_hash, int workers)
{
  auto cache = MapDistanceCache();
  cache.metadata.map_name = map_name;
  cache.metadata.map_hash = map_hash;
  cache.metadata.width = graph.width;
  cache.metadata.height = graph.height;
  cache.metadata.traversable_count = graph.size();
  cache.distances.resize(graph.size());
  const auto thread_count = std::max(
      1, std::min(graph.size(),
                  workers > 0 ? workers
                              : static_cast<int>(std::max(
                                    1u, std::thread::hardware_concurrency()))));
  auto next_source = std::atomic<int>(0);
  auto pool = std::vector<std::thread>();
  pool.reserve(thread_count);
  for (auto worker = 0; worker < thread_count; ++worker) {
    pool.emplace_back([&] {
      while (true) {
        const auto source = next_source.fetch_add(1);
        if (source >= graph.size()) break;
        cache.distances[source] = bfs_distances_from(graph, graph.V[source]);
      }
    });
  }
  for (auto& thread : pool) thread.join();
  return cache;
}

MapDistanceCache load_or_build_map_distance_cache(
    const std::filesystem::path& map_path,
    const std::filesystem::path& cache_path, int verbose, int workers)
{
  const auto graph = Graph(map_path.string());
  const auto map_hash = hash_file(map_path);
  auto expected = MapDistanceCacheMetadata();
  expected.map_name = map_path.filename().string();
  expected.map_hash = map_hash;
  expected.width = graph.width;
  expected.height = graph.height;
  expected.traversable_count = graph.size();

  auto cache = MapDistanceCache();
  if (load_map_distance_cache(cache_path, expected, cache)) {
    info(1, verbose, "loaded map distance cache: ", cache_path.string(), "\n");
    return cache;
  }

  const auto estimated_bytes = estimate_distance_cache_bytes(graph.size());
  info(1, verbose, "building map distance cache\n");
  info(1, verbose, "traversable cells: ", graph.size(), "\n");
  info(1, verbose, "estimated cache bytes: ", estimated_bytes, "\n");
  info(1, verbose, "estimated memory bytes: ", estimated_bytes, "\n");
  info(1, verbose, "cache path: ", cache_path.string(), "\n");

  constexpr auto max_cache_bytes = 8ULL * 1024ULL * 1024ULL * 1024ULL;
  if (estimated_bytes > max_cache_bytes) {
    throw std::runtime_error(
        "all-pairs distance cache is too large for eager build; traversable=" +
        std::to_string(graph.size()) + " estimated_matrix_bytes=" +
        std::to_string(estimated_bytes) + " cache_path=" + cache_path.string() +
        " suggested_alternative=lazy BFS cache or region-to-region cache");
  }

  cache = build_map_distance_cache(graph, expected.map_name, expected.map_hash,
                                   workers);
  save_map_distance_cache(cache_path, cache);
  return cache;
}

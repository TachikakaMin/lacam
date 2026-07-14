#include <chrono>
#include <filesystem>
#include <iostream>
#include <lacam.hpp>
#include <thread>

int main(int argc, char** argv)
{
  if (argc < 3 || argc > 4) {
    std::cerr << "usage: map_distance_precompute MAP CACHE [WORKERS=auto]\n";
    return 2;
  }
  const auto map_path = std::filesystem::path(argv[1]);
  const auto cache_path = std::filesystem::path(argv[2]);
  const auto workers =
      argc == 4
          ? std::stoi(argv[3])
          : static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
  const auto started = std::chrono::steady_clock::now();
  try {
    const auto graph = Graph(map_path.string());
    const auto map_hash = hash_file(map_path);
    const auto metadata =
        MapDistanceCacheMetadata{map_path.filename().string(), map_hash,
                                 graph.width, graph.height, graph.size()};
    auto probe = MapDistanceRows();
    if (load_map_distance_rows(cache_path, metadata, {0}, probe)) {
      std::cout << "cache_status=loaded\n";
    } else {
      auto cache = build_map_distance_cache(graph, metadata.map_name,
                                            metadata.map_hash, workers);
      save_map_distance_cache(cache_path, cache);
      std::cout << "cache_status=built\n";
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started);
    std::cout << "map=" << map_path << "\n"
              << "cache=" << cache_path << "\n"
              << "traversable=" << graph.size() << "\n"
              << "matrix_bytes=" << estimate_distance_cache_bytes(graph.size())
              << "\nworkers=" << workers << "\n"
              << "elapsed_ms=" << elapsed.count() << "\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}

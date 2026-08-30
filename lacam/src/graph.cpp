#include "../include/graph.hpp"

Vertex::Vertex(int _id, int _index)
    : id(_id), index(_index), neighbor(Vertices())
{
}

Graph::Graph() : V(Vertices()), width(0), height(0) {}
Graph::~Graph()
{
  for (auto& v : V)
    if (v != nullptr) delete v;
  V.clear();
}

// to load graph
static const std::regex r_height = std::regex(R"(height\s(\d+))");
static const std::regex r_width = std::regex(R"(width\s(\d+))");
static const std::regex r_map = std::regex(R"(map)");

Graph::Graph(const std::string& filename) : V(Vertices()), width(0), height(0)
{
  std::ifstream file(filename);
  if (!file) {
    std::cout << "file " << filename << " is not found." << std::endl;
    return;
  }
  std::string line;
  std::smatch results;

  // read fundamental graph parameters
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_height)) {
      height = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_width)) {
      width = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_map)) break;
  }

  // collect map rows, then build (same order as before the extraction)
  auto rows = std::vector<std::string>();
  while (getline(file, line) && (int)rows.size() < height) {
    // for CRLF coding
    if (!line.empty() && *(line.end() - 1) == 0x0d) line.pop_back();
    rows.push_back(line);
  }
  file.close();
  build_from_rows(rows);
}

Graph::Graph(const std::vector<std::string>& rows)
    : V(Vertices()), width(0), height(0)
{
  height = rows.size();
  for (const auto& row : rows) width = std::max(width, (int)row.size());
  build_from_rows(rows);
}

void Graph::build_from_rows(const std::vector<std::string>& rows)
{
  U = Vertices(width * height, nullptr);

  // create vertices
  for (int y = 0; y < height && y < (int)rows.size(); ++y) {
    const auto& line = rows[y];
    for (int x = 0; x < width && x < (int)line.size(); ++x) {
      char s = line[x];
      if (is_map_wall_char(s)) continue;  // object (shared rule)
      auto index = width * y + x;
      auto v = new Vertex(V.size(), index);
      V.push_back(v);
      U[index] = v;
    }
  }

  // create edges
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto v = U[width * y + x];
      if (v == nullptr) continue;
      // left
      if (x > 0) {
        auto u = U[width * y + (x - 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // right
      if (x < width - 1) {
        auto u = U[width * y + (x + 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // up
      if (y < height - 1) {
        auto u = U[width * (y + 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // down
      if (y > 0) {
        auto u = U[width * (y - 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
    }
  }
}

int Graph::size() const { return V.size(); }

bool is_same_config(const Config& C1, const Config& C2)
{
  const auto N = C1.size();
  for (size_t i = 0; i < N; ++i) {
    if (C1[i]->id != C2[i]->id) return false;
  }
  return true;
}

uint ConfigHasher::operator()(const Config& C) const
{
  uint hash = C.size();
  for (auto& v : C) {
    hash ^= v->id + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  }
  return hash;
}

std::ostream& operator<<(std::ostream& os, const Vertex* v)
{
  os << v->index;
  return os;
}

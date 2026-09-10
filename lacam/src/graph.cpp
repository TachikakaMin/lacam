#include "../include/graph.hpp"
#include "../include/dd_carrier.hpp"

Vertex::Vertex(int _id, int _index)
    : id(_id),
      index(_index),
      neighbor(Vertices()),
      predecessor(Vertices())
{
}

Graph::Graph()
    : V(Vertices()),
      width(0),
      height(0),
      explicit_adjacency(false)
{
}
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

Graph::Graph(const std::string& filename)
    : V(Vertices()),
      width(0),
      height(0),
      explicit_adjacency(false)
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
    : V(Vertices()),
      width(0),
      height(0),
      explicit_adjacency(false)
{
  height = rows.size();
  for (const auto& row : rows) width = std::max(width, (int)row.size());
  build_from_rows(rows);
}

Graph::Graph(const DDGrid& grid)
    : V(Vertices()),
      width(grid.width),
      height(grid.height),
      explicit_adjacency(grid.uses_explicit_adjacency())
{
  U = Vertices(width * height, nullptr);
  for (int cell = 0; cell < grid.size(); ++cell) {
    if (grid.is_wall(cell)) continue;
    auto* vertex = new Vertex(V.size(), cell);
    V.push_back(vertex);
    U[cell] = vertex;
  }
  if (!explicit_adjacency) {
    std::vector<std::string> rows(
        height, std::string(width, '.'));
    for (int cell = 0; cell < grid.size(); ++cell)
      if (grid.is_wall(cell))
        rows[grid.row(cell)][grid.col(cell)] = '@';
    for (auto* vertex : V) delete vertex;
    V.clear();
    U.clear();
    build_from_rows(rows);
    return;
  }
  for (int from = 0; from < grid.size(); ++from) {
    auto* source = U[from];
    if (source == nullptr) continue;
    for (const int to : grid.outgoing(from)) {
      auto* destination = U[to];
      if (destination == nullptr)
        throw std::invalid_argument(
            "Graph: adjacency touches a wall");
      source->neighbor.push_back(destination);
      destination->predecessor.push_back(source);
    }
  }
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
        if (u != nullptr) {
          v->neighbor.push_back(u);
          u->predecessor.push_back(v);
        }
      }
      // right
      if (x < width - 1) {
        auto u = U[width * y + (x + 1)];
        if (u != nullptr) {
          v->neighbor.push_back(u);
          u->predecessor.push_back(v);
        }
      }
      // up
      if (y < height - 1) {
        auto u = U[width * (y + 1) + x];
        if (u != nullptr) {
          v->neighbor.push_back(u);
          u->predecessor.push_back(v);
        }
      }
      // down
      if (y > 0) {
        auto u = U[width * (y - 1) + x];
        if (u != nullptr) {
          v->neighbor.push_back(u);
          u->predecessor.push_back(v);
        }
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

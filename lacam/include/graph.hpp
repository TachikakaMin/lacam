/*
 * graph definition
 */
#pragma once
#include "utils.hpp"

struct Vertex {
  const int id;     // index for V in Graph
  const int index;  // index for U (width * y + x) in Graph
  std::vector<Vertex*> neighbor;

  Vertex(int _id, int _index);
};
using Vertices = std::vector<Vertex*>;
using Config = std::vector<Vertex*>;  // locations for all agents

struct Graph {
  Vertices V;  // without nullptr
  Vertices U;  // with nullptr, i.e., |U| = width * height
  int width;   // grid width
  int height;  // grid height
  Graph();
  Graph(const std::string& filename);  // taking map filename
  // inline map rows (carrier integration M1): same wall rule and the same
  // vertex/edge construction order as the file loader
  explicit Graph(const std::vector<std::string>& rows);
  ~Graph();

  int size() const;  // the number of vertices, |V|

 private:
  // shared vertex/edge construction (file loader and inline rows)
  void build_from_rows(const std::vector<std::string>& rows);
};

bool is_same_config(
    const Config& C1,
    const Config& C2);  // check equivalence of two configurations

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config& C) const;
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);

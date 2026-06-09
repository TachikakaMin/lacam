#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(Graph, load_graph)
{
  const std::string filename = "./assets/random-32-32-10.map";
  auto G = Graph(filename);
  ASSERT_EQ(G.size(), 922);
  ASSERT_EQ(G.V[0]->neighbor.size(), 2);
  ASSERT_EQ(G.V[0]->neighbor[0]->id, 1);
  ASSERT_EQ(G.V[0]->neighbor[1]->id, 28);
  ASSERT_EQ(G.width, 32);
  ASSERT_EQ(G.height, 32);
}

TEST(Graph, preserve_symbotic_cell_types)
{
  const std::string filename = "./tests/assets/symbotic.map";
  auto G = Graph(filename);

  const auto a_index = G.width * 2 + 1;
  const auto o_index = G.width * 27 + 1;
  const auto i_index = G.width * 27 + 19;
  const auto wall_index = G.width * 2 + 0;

  ASSERT_FALSE(G.is_traversable(wall_index));
  ASSERT_EQ(G.cell_type(wall_index), '@');

  ASSERT_TRUE(G.is_traversable(a_index));
  ASSERT_TRUE(G.is_traversable(o_index));
  ASSERT_TRUE(G.is_traversable(i_index));
  ASSERT_EQ(G.cell_type(a_index), 'a');
  ASSERT_EQ(G.cell_type(o_index), 'o');
  ASSERT_EQ(G.cell_type(i_index), 'i');

  ASSERT_GT(G.vertices_of_type('a').size(), 0);
  ASSERT_GT(G.vertices_of_type('o').size(), 0);
  ASSERT_GT(G.vertices_of_type('i').size(), 0);
}

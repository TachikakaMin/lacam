// Golden transition corpus — C++ side (debug.md round-2 P1-8).
//
// Shared corpus: tests/fixtures/golden/*.{yaml,trans} (see the Python twin
// benchmark/tests/test_golden_corpus.py).  Sequential protocol: `legal`
// lines must be accepted by apply_ops and advance the state; `illegal`
// lines must be rejected and leave the state untouched.  Any divergence
// between the two validators shows up as exactly one side failing.
#include <dd_carrier.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

struct TransLine {
  bool legal;
  std::vector<Op> ops;
  std::string raw;
};

std::vector<TransLine> parse_trans(const DDGrid& g, const std::string& path)
{
  std::ifstream in(path);
  EXPECT_TRUE(in.good()) << path;
  std::vector<TransLine> out;
  std::string raw;
  while (std::getline(in, raw)) {
    std::string line = raw.substr(0, raw.find('#'));
    // trim
    const auto a = line.find_first_not_of(" \t");
    if (a == std::string::npos) continue;
    std::istringstream ss(line);
    std::string verdict;
    ss >> verdict;
    std::string rest;
    std::getline(ss, rest);
    TransLine tl;
    tl.legal = verdict == "legal";
    tl.raw = raw;
    std::istringstream toks(rest);
    std::string tok;
    std::vector<std::string> parts;
    {
      std::string acc;
      std::istringstream rs(rest);
      while (std::getline(rs, acc, ';')) parts.push_back(acc);
    }
    for (auto& p : parts) {
      std::istringstream ps(p);
      std::string kind;
      ps >> kind;
      if (kind == "w") {
        tl.ops.push_back(Op::make_wait());
      } else if (kind == "m") {
        int r, c;
        ps >> r >> c;
        tl.ops.push_back(Op::make_move(g.idx(r, c)));
      } else if (kind == "l") {
        tl.ops.push_back(Op::make_lift());
      } else if (kind == "d") {
        tl.ops.push_back(Op::make_drop());
      } else {
        ADD_FAILURE() << "bad token '" << kind << "' in " << path;
      }
    }
    out.push_back(std::move(tl));
  }
  return out;
}

}  // namespace

TEST(dd_golden, corpus_agreement)
{
  const std::string dir = std::string(DD_TEST_DIR) + "/fixtures/golden";
  const std::vector<std::string> cases = {"g1_lower_rules", "g2_upper_rules",
                                          "g3_cycle2x2"};
  int total_lines = 0;
  for (const auto& name : cases) {
    DDInstance ins = load_dd_instance(dir + "/" + name + ".yaml");
    PhysConfig X = initial_phys_config(ins);
    const auto lines = parse_trans(ins.grid, dir + "/" + name + ".trans");
    EXPECT_FALSE(lines.empty()) << name;
    int k = 0;
    for (const auto& tl : lines) {
      auto res = apply_ops(ins, X, tl.ops);
      if (tl.legal) {
        ASSERT_TRUE(res.has_value())
            << name << ":" << k << " expected legal, C++ validator rejected"
            << "\n  line: " << tl.raw;
        X = *res;
      } else {
        ASSERT_FALSE(res.has_value())
            << name << ":" << k << " expected illegal, C++ validator "
            << "accepted\n  line: " << tl.raw;
      }
      ++k;
      ++total_lines;
    }
  }
  EXPECT_GE(total_lines, 20) << "corpus must stay non-trivial";
}

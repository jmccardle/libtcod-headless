
#include <catch2/catch_all.hpp>
#include <libtcod/bsp.hpp>

TEST_CASE("BSP traversal") {
  auto root = TCODBsp{};
  root.splitOnce(0, 0);
  root.getLeft()->splitOnce(0, 0);

  auto pre_order = std::vector<TCODBsp*>{};
  REQUIRE(root.traversePreOrder([&](TCODBsp& node) { pre_order.push_back(&node); }));
  REQUIRE(pre_order.at(0) == &root);
  REQUIRE(pre_order.at(1) == root.getLeft());
  REQUIRE(pre_order.at(2) == root.getLeft()->getLeft());
  REQUIRE(pre_order.at(3) == root.getLeft()->getRight());
  REQUIRE(pre_order.at(4) == root.getRight());
  REQUIRE(root.traverseInOrder([&](TCODBsp&) { return true; }));
  REQUIRE(root.traversePostOrder([&](TCODBsp&) { return true; }));
  REQUIRE(root.traverseLevelOrder([&](TCODBsp&) { return true; }));
  REQUIRE(root.traverseInvertedLevelOrder([&](TCODBsp&) { return true; }));
};

static bool bsp_record_node(TCOD_bsp_t* node, void* userData) {
  static_cast<std::vector<TCOD_bsp_t*>*>(userData)->push_back(node);
  return true;
}

TEST_CASE("BSP C level order traversal") {
  TCOD_bsp_t* root = TCOD_bsp_new_with_size(0, 0, 10, 10);
  TCOD_bsp_split_once(root, false, 5);
  TCOD_bsp_split_once(TCOD_bsp_left(root), true, 5);
  TCOD_bsp_t* left = TCOD_bsp_left(root);
  const auto expected = std::vector{root, left, TCOD_bsp_right(root), TCOD_bsp_left(left), TCOD_bsp_right(left)};

  auto level_order = std::vector<TCOD_bsp_t*>{};
  REQUIRE(TCOD_bsp_traverse_level_order(root, bsp_record_node, &level_order));
  REQUIRE(level_order == expected);

  auto inverted_level_order = std::vector<TCOD_bsp_t*>{};
  REQUIRE(TCOD_bsp_traverse_inverted_level_order(root, bsp_record_node, &inverted_level_order));
  REQUIRE(inverted_level_order == std::vector<TCOD_bsp_t*>(expected.rbegin(), expected.rend()));
  TCOD_bsp_delete(root);
}

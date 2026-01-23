#include <catch2/catch_all.hpp>
#include <cmath>

#include "libtcod/heightmap.hpp"

TEST_CASE("TCODHeightmap") {
  auto hm = TCODHeightMap(0, 0);  // Test zero size
  hm = TCODHeightMap(1, 1);  // Test assign new shape
  REQUIRE(hm.getValue(0, 0) == 0);
  hm.setValue(0, 0, 1.0f);  // Should be zeroed
  REQUIRE(hm.getValue(0, 0) == 1.0f);

  auto hm2 = TCODHeightMap(1, 1);
  hm2.setValue(0, 0, 3.0f);
  hm = hm2;  // Test same-size copy
  REQUIRE(hm.getValue(0, 0) == 3.0f);

  hm2 = TCODHeightMap(2, 3);
  hm2.setValue(0, 0, 4.0f);
  hm = std::move(hm2);  // Test move
  REQUIRE(hm.getValue(0, 0) == 4.0f);
  REQUIRE(hm.w == 2);
  REQUIRE(hm.h == 3);

  hm.clear();
  REQUIRE(hm.getValue(0, 0) == 0.0f);
}

TEST_CASE("tcod::heightmap_lerp") {
  TCODHeightMap a(2, 2);
  TCODHeightMap b(2, 2);

  a.setValue(0, 0, 0.0f);
  b.setValue(0, 0, 10.0f);

  auto result = tcod::heightmap_lerp(a, b, 0.5f);
  REQUIRE(result.w == 2);
  REQUIRE(result.h == 2);
  REQUIRE(std::abs(result.getValue(0, 0) - 5.0f) < 0.001f);

  // coef = 0.0 should return a
  result = tcod::heightmap_lerp(a, b, 0.0f);
  REQUIRE(std::abs(result.getValue(0, 0) - 0.0f) < 0.001f);

  // coef = 1.0 should return b
  result = tcod::heightmap_lerp(a, b, 1.0f);
  REQUIRE(std::abs(result.getValue(0, 0) - 10.0f) < 0.001f);
}

TEST_CASE("tcod::heightmap_add") {
  TCODHeightMap a(2, 2);
  TCODHeightMap b(2, 2);

  a.setValue(0, 0, 3.0f);
  b.setValue(0, 0, 7.0f);

  auto result = tcod::heightmap_add(a, b);
  REQUIRE(result.w == 2);
  REQUIRE(result.h == 2);
  REQUIRE(std::abs(result.getValue(0, 0) - 10.0f) < 0.001f);
}

TEST_CASE("tcod::heightmap_multiply") {
  TCODHeightMap a(2, 2);
  TCODHeightMap b(2, 2);

  a.setValue(0, 0, 3.0f);
  b.setValue(0, 0, 4.0f);

  auto result = tcod::heightmap_multiply(a, b);
  REQUIRE(result.w == 2);
  REQUIRE(result.h == 2);
  REQUIRE(std::abs(result.getValue(0, 0) - 12.0f) < 0.001f);
}

TEST_CASE("tcod::heightmap functions with mismatched sizes") {
  TCODHeightMap a(2, 2);
  TCODHeightMap b(3, 3);

  // Should return empty heightmap on size mismatch
  auto result = tcod::heightmap_lerp(a, b, 0.5f);
  REQUIRE(result.w == 0);
  REQUIRE(result.h == 0);

  result = tcod::heightmap_add(a, b);
  REQUIRE(result.w == 0);
  REQUIRE(result.h == 0);

  result = tcod::heightmap_multiply(a, b);
  REQUIRE(result.w == 0);
  REQUIRE(result.h == 0);
}

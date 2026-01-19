#include <catch2/catch_all.hpp>
#include <cfloat>
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

TEST_CASE("TCOD_heightmap_kernel_transform_hm identity kernel", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(3, 3);
  TCOD_heightmap_t* dst = TCOD_heightmap_new(3, 3);

  // Set center to 1, all others to 0
  TCOD_heightmap_set_value(src, 1, 1, 1.0f);

  // Identity kernel (just center)
  const int dx[] = {0};
  const int dy[] = {0};
  const float weight[] = {1.0f};

  TCOD_heightmap_kernel_transform_hm(src, dst, 1, dx, dy, weight, -FLT_MAX, FLT_MAX);
  REQUIRE(TCOD_heightmap_get_value(dst, 1, 1) == 1.0f);
  REQUIRE(TCOD_heightmap_get_value(dst, 0, 0) == 0.0f);

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dst);
}

TEST_CASE("TCOD_heightmap_kernel_transform_hm vs in-place differ", "[heightmap]") {
  // Demonstrate that the new function produces different (correct) results
  TCOD_heightmap_t* src = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dst_correct = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dst_inplace = TCOD_heightmap_new(5, 5);

  // Fill with gradient
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      float val = static_cast<float>(x + y);
      TCOD_heightmap_set_value(src, x, y, val);
      TCOD_heightmap_set_value(dst_inplace, x, y, val);
    }
  }

  // Simple averaging kernel (horizontal neighbors)
  const int dx[] = {-1, 0, 1};
  const int dy[] = {0, 0, 0};
  const float weight[] = {1, 1, 1};

  // Apply both methods
  TCOD_heightmap_kernel_transform_hm(src, dst_correct, 3, dx, dy, weight, -FLT_MAX, FLT_MAX);
  TCOD_heightmap_kernel_transform(dst_inplace, 3, dx, dy, weight, -FLT_MAX, FLT_MAX);

  // Results should differ due to in-place modification
  bool any_differ = false;
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      if (TCOD_heightmap_get_value(dst_correct, x, y) != TCOD_heightmap_get_value(dst_inplace, x, y)) {
        any_differ = true;
      }
    }
  }
  REQUIRE(any_differ);  // They SHOULD differ

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dst_correct);
  TCOD_heightmap_delete(dst_inplace);
}

TEST_CASE("TCOD_heightmap_kernel_transform_hm level filtering", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(3, 3);
  TCOD_heightmap_t* dst = TCOD_heightmap_new(3, 3);

  // Set values: corners=0.5, center=2.0
  for (int y = 0; y < 3; y++) {
    for (int x = 0; x < 3; x++) {
      TCOD_heightmap_set_value(src, x, y, 0.5f);
    }
  }
  TCOD_heightmap_set_value(src, 1, 1, 2.0f);

  // Average kernel
  const int dx[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
  const int dy[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
  const float weight[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

  // Only transform cells with value >= 1.0 (just the center)
  TCOD_heightmap_kernel_transform_hm(src, dst, 9, dx, dy, weight, 1.0f, FLT_MAX);

  // Center should be transformed (average of 8*0.5 + 1*2.0 = 6.0 / 9)
  REQUIRE(TCOD_heightmap_get_value(dst, 1, 1) == Catch::Approx(6.0f / 9.0f));
  // Corners should be copied unchanged
  REQUIRE(TCOD_heightmap_get_value(dst, 0, 0) == 0.5f);

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dst);
}

TEST_CASE("TCOD_heightmap_convolve3x3 blur", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dst = TCOD_heightmap_new(5, 5);

  // Single spike in center
  TCOD_heightmap_set_value(src, 2, 2, 9.0f);

  // Box blur kernel
  const float kernel[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

  TCOD_heightmap_convolve3x3(src, dst, kernel, true);

  // Center should now be 1.0 (9/9 neighbors counted)
  REQUIRE(TCOD_heightmap_get_value(dst, 2, 2) == Catch::Approx(1.0f));
  // Adjacent cells should also be 1.0
  REQUIRE(TCOD_heightmap_get_value(dst, 1, 2) == Catch::Approx(1.0f));
  REQUIRE(TCOD_heightmap_get_value(dst, 2, 1) == Catch::Approx(1.0f));

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dst);
}

TEST_CASE("TCOD_heightmap_convolve3x3 no normalize", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dst = TCOD_heightmap_new(5, 5);

  // Fill with 1.0
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      TCOD_heightmap_set_value(src, x, y, 1.0f);
    }
  }

  // Sobel X kernel (sums to 0)
  const float sobel_x[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};

  TCOD_heightmap_convolve3x3(src, dst, sobel_x, false);

  // Uniform input should give 0 gradient
  REQUIRE(TCOD_heightmap_get_value(dst, 2, 2) == Catch::Approx(0.0f));

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dst);
}

TEST_CASE("TCOD_heightmap_gradient flat surface", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dx = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dy = TCOD_heightmap_new(5, 5);

  // Flat surface (all zeros)
  TCOD_heightmap_gradient(src, dx, dy);

  // Gradient should be zero everywhere
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      REQUIRE(TCOD_heightmap_get_value(dx, x, y) == Catch::Approx(0.0f));
      REQUIRE(TCOD_heightmap_get_value(dy, x, y) == Catch::Approx(0.0f));
    }
  }

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dx);
  TCOD_heightmap_delete(dy);
}

TEST_CASE("TCOD_heightmap_gradient slope in x direction", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dx = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dy = TCOD_heightmap_new(5, 5);

  // Create a plane: z = x (gradient should be (1, 0) everywhere interior)
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      TCOD_heightmap_set_value(src, x, y, static_cast<float>(x));
    }
  }

  TCOD_heightmap_gradient(src, dx, dy);

  // Interior point should have dx = 1, dy = 0
  REQUIRE(TCOD_heightmap_get_value(dx, 2, 2) == Catch::Approx(1.0f));
  REQUIRE(TCOD_heightmap_get_value(dy, 2, 2) == Catch::Approx(0.0f));

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dx);
  TCOD_heightmap_delete(dy);
}

TEST_CASE("TCOD_heightmap_gradient slope in y direction", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dx = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dy = TCOD_heightmap_new(5, 5);

  // Create a plane: z = y
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      TCOD_heightmap_set_value(src, x, y, static_cast<float>(y));
    }
  }

  TCOD_heightmap_gradient(src, dx, dy);

  // Interior point should have dx = 0, dy = 1
  REQUIRE(TCOD_heightmap_get_value(dx, 2, 2) == Catch::Approx(0.0f));
  REQUIRE(TCOD_heightmap_get_value(dy, 2, 2) == Catch::Approx(1.0f));

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dx);
  TCOD_heightmap_delete(dy);
}

TEST_CASE("TCOD_heightmap_gradient null output parameters", "[heightmap]") {
  TCOD_heightmap_t* src = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_t* dx = TCOD_heightmap_new(5, 5);

  // Fill source
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      TCOD_heightmap_set_value(src, x, y, static_cast<float>(x));
    }
  }

  // Should work with NULL dy
  TCOD_heightmap_gradient(src, dx, NULL);
  REQUIRE(TCOD_heightmap_get_value(dx, 2, 2) == Catch::Approx(1.0f));

  // Should work with NULL dx
  TCOD_heightmap_t* dy = TCOD_heightmap_new(5, 5);
  TCOD_heightmap_gradient(src, NULL, dy);
  REQUIRE(TCOD_heightmap_get_value(dy, 2, 2) == Catch::Approx(0.0f));

  TCOD_heightmap_delete(src);
  TCOD_heightmap_delete(dx);
  TCOD_heightmap_delete(dy);
}

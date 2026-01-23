#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>

#include "libtcod/heightmap.h"
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

TEST_CASE("TCOD_heightmap_kernel_transform_out") {
  // Test that kernel transform produces correct results
  auto* src = TCOD_heightmap_new(3, 3);
  auto* dst = TCOD_heightmap_new(3, 3);

  // Set center cell to 1, all others to 0
  TCOD_heightmap_set_value(src, 1, 1, 1.0f);

  // Identity kernel (just center)
  const int dx_identity[] = {0};
  const int dy_identity[] = {0};
  const float weight_identity[] = {1.0f};

  TCOD_heightmap_kernel_transform_out(src, dst, 1, dx_identity, dy_identity, weight_identity);
  REQUIRE(TCOD_heightmap_get_value(dst, 1, 1) == 1.0f);
  REQUIRE(TCOD_heightmap_get_value(dst, 0, 0) == 0.0f);

  // 3x3 box blur kernel
  const int dx_blur[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
  const int dy_blur[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
  const float weight_blur[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

  TCOD_heightmap_kernel_transform_out(src, dst, 9, dx_blur, dy_blur, weight_blur);
  // Center should be 1/9 (one cell with value 1, 9 cells total)
  REQUIRE(std::abs(TCOD_heightmap_get_value(dst, 1, 1) - 1.0f / 9.0f) < 0.001f);

  TCOD_heightmap_delete(dst);
  TCOD_heightmap_delete(src);
}

TEST_CASE("TCOD_heightmap_convolve3x3") {
  auto* src = TCOD_heightmap_new(3, 3);
  auto* dst = TCOD_heightmap_new(3, 3);

  // Set center cell to 1
  TCOD_heightmap_set_value(src, 1, 1, 1.0f);

  // Box blur kernel (all 1s)
  const float kernel_blur[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

  TCOD_heightmap_convolve3x3(src, dst, kernel_blur);
  // Center should be 1/9
  REQUIRE(std::abs(TCOD_heightmap_get_value(dst, 1, 1) - 1.0f / 9.0f) < 0.001f);

  // Gaussian blur kernel
  const float kernel_gaussian[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};
  TCOD_heightmap_convolve3x3(src, dst, kernel_gaussian);
  // Center should be 4/16 = 0.25 (weight 4 on the center value of 1)
  REQUIRE(std::abs(TCOD_heightmap_get_value(dst, 1, 1) - 4.0f / 16.0f) < 0.001f);

  TCOD_heightmap_delete(dst);
  TCOD_heightmap_delete(src);
}

TEST_CASE("Heightmap convolution sparse vs dense equivalence") {
  // Verify that sparse and dense 3x3 convolutions produce identical results
  const int size = 32;
  auto* src = TCOD_heightmap_new(size, size);
  auto* dst_sparse = TCOD_heightmap_new(size, size);
  auto* dst_dense = TCOD_heightmap_new(size, size);

  // Fill with pattern
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      TCOD_heightmap_set_value(src, x, y, static_cast<float>(x + y * size) / (size * size));
    }
  }

  // Same kernel in both formats
  const int dx[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
  const int dy[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
  const float weight[] = {1, 2, 1, 2, 4, 2, 1, 2, 1};  // Gaussian
  const float kernel[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};

  TCOD_heightmap_kernel_transform_out(src, dst_sparse, 9, dx, dy, weight);
  TCOD_heightmap_convolve3x3(src, dst_dense, kernel);

  // Results should be identical
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      const float sparse_val = TCOD_heightmap_get_value(dst_sparse, x, y);
      const float dense_val = TCOD_heightmap_get_value(dst_dense, x, y);
      REQUIRE(std::abs(sparse_val - dense_val) < 0.0001f);
    }
  }

  TCOD_heightmap_delete(dst_dense);
  TCOD_heightmap_delete(dst_sparse);
  TCOD_heightmap_delete(src);
}

TEST_CASE("Heightmap convolution benchmark", "[!benchmark]") {
  // Benchmark comparing sparse vs dense 3x3 convolution
  // Run with: ./unittest "[!benchmark]" to include benchmarks
  const int size = 256;
  const int iterations = 100;

  auto* src = TCOD_heightmap_new(size, size);
  auto* dst = TCOD_heightmap_new(size, size);

  // Fill with random-ish data
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      TCOD_heightmap_set_value(src, x, y, static_cast<float>((x * 17 + y * 31) % 100) / 100.0f);
    }
  }

  // Sparse kernel arrays
  const int dx[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
  const int dy[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
  const float weight[] = {1, 2, 1, 2, 4, 2, 1, 2, 1};

  // Dense kernel
  const float kernel[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};

  BENCHMARK("Sparse 3x3 convolution") {
    for (int i = 0; i < iterations; i++) {
      TCOD_heightmap_kernel_transform_out(src, dst, 9, dx, dy, weight);
    }
    return TCOD_heightmap_get_value(dst, size / 2, size / 2);  // Prevent optimization
  };

  BENCHMARK("Dense 3x3 convolution") {
    for (int i = 0; i < iterations; i++) {
      TCOD_heightmap_convolve3x3(src, dst, kernel);
    }
    return TCOD_heightmap_get_value(dst, size / 2, size / 2);  // Prevent optimization
  };

  TCOD_heightmap_delete(dst);
  TCOD_heightmap_delete(src);
}

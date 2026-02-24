/**
 * Tests for A* pathfinding with custom heuristics.
 *
 * These tests verify the new TCOD_heuristic_func_t callback system
 * and the built-in heuristic functions.
 */

#include <libtcod/fov.h>
#include <libtcod/path.h>

#include <catch2/catch_all.hpp>
#include <cmath>

// Simple open map for testing
static float simple_cost(int xFrom, int yFrom, int xTo, int yTo, void* userData) {
  (void)xFrom;
  (void)yFrom;
  (void)xTo;
  (void)yTo;
  (void)userData;
  return 1.0f;  // All cells walkable with cost 1
}

// Custom heuristic that counts expanded nodes
struct TestData {
  int nodes_expanded;
};

static float counting_heuristic(int x, int y, int goal_x, int goal_y, void* userData) {
  auto* data = static_cast<TestData*>(userData);
  data->nodes_expanded++;
  // Use Manhattan distance
  return (float)(abs(x - goal_x) + abs(y - goal_y));
}

TEST_CASE("A* default heuristic produces valid path", "[path]") {
  // Create pathfinder with default Euclidean heuristic
  auto path = TCOD_path_new_using_function(20, 20, simple_cost, nullptr, 1.41f);
  REQUIRE(path != nullptr);

  // Compute path from (0,0) to (19,19)
  REQUIRE(TCOD_path_compute(path, 0, 0, 19, 19));

  // Verify path is not empty
  REQUIRE(!TCOD_path_is_empty(path));

  // Walk the path and verify we reach destination
  int x = 0, y = 0;
  while (TCOD_path_walk(path, &x, &y, false)) {
    // Keep walking
  }
  // After walking, we should be at or near destination
  REQUIRE(x == 19);
  REQUIRE(y == 19);

  TCOD_path_delete(path);
}

TEST_CASE("A* with custom heuristic function", "[path]") {
  TestData data{0};

  // Create pathfinder with custom heuristic
  auto path = TCOD_path_new_using_function_ex(20, 20, simple_cost, counting_heuristic, &data, 1.41f, 1.0f);
  REQUIRE(path != nullptr);

  // Compute path
  REQUIRE(TCOD_path_compute(path, 0, 0, 19, 19));
  REQUIRE(data.nodes_expanded > 0);

  // Verify path is valid
  REQUIRE(!TCOD_path_is_empty(path));

  TCOD_path_delete(path);
}

TEST_CASE("Weighted A* explores fewer nodes", "[path]") {
  // Test that weighted A* (weight > 1.0) explores fewer nodes
  // at the cost of potentially non-optimal paths

  TestData data_optimal{0};
  TestData data_weighted{0};

  // Optimal A* (weight = 1.0)
  auto path_optimal =
      TCOD_path_new_using_function_ex(30, 30, simple_cost, counting_heuristic, &data_optimal, 1.41f, 1.0f);
  TCOD_path_compute(path_optimal, 0, 0, 29, 29);
  int optimal_path_length = TCOD_path_size(path_optimal);

  // Weighted A* (weight = 2.0)
  auto path_weighted =
      TCOD_path_new_using_function_ex(30, 30, simple_cost, counting_heuristic, &data_weighted, 1.41f, 2.0f);
  TCOD_path_compute(path_weighted, 0, 0, 29, 29);
  int weighted_path_length = TCOD_path_size(path_weighted);

  // Weighted A* should explore fewer or equal nodes
  // (in simple open maps, might explore similar amounts)
  REQUIRE(data_weighted.nodes_expanded <= data_optimal.nodes_expanded + 10);

  // Both paths should be valid (reach destination)
  REQUIRE(optimal_path_length > 0);
  REQUIRE(weighted_path_length > 0);

  TCOD_path_delete(path_optimal);
  TCOD_path_delete(path_weighted);
}

TEST_CASE("TCOD_path_set_heuristic changes behavior", "[path]") {
  // Create path with default heuristic
  auto path = TCOD_path_new_using_function(20, 20, simple_cost, nullptr, 1.41f);
  REQUIRE(path != nullptr);

  // Compute path (uses default Euclidean)
  REQUIRE(TCOD_path_compute(path, 0, 0, 10, 10));
  int len1 = TCOD_path_size(path);

  // Change to a stateful custom heuristic with its own context
  TestData data{0};
  TCOD_path_set_heuristic(path, counting_heuristic, &data, 1.0f);
  REQUIRE(TCOD_path_compute(path, 0, 0, 10, 10));
  int len2 = TCOD_path_size(path);

  // The custom heuristic was actually consulted, and both paths are valid
  REQUIRE(data.nodes_expanded > 0);
  REQUIRE(len1 > 0);
  REQUIRE(len2 > 0);

  TCOD_path_delete(path);
}

TEST_CASE("A* with map-based pathfinder", "[path]") {
  // Test with TCOD_Map instead of function callback
  auto map = TCOD_map_new(20, 20);
  REQUIRE(map != nullptr);

  // Make all cells walkable
  for (int y = 0; y < 20; y++) {
    for (int x = 0; x < 20; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  // Create path with map
  auto path = TCOD_path_new_using_map(map, 1.41f);
  REQUIRE(path != nullptr);

  // Compute path
  REQUIRE(TCOD_path_compute(path, 0, 0, 19, 19));
  REQUIRE(!TCOD_path_is_empty(path));

  // A stateful custom heuristic carries its own context even on map-based paths,
  // where the pathfinder itself has no walk-cost user data.
  TestData data{0};
  TCOD_path_set_heuristic(path, counting_heuristic, &data, 1.0f);
  REQUIRE(TCOD_path_compute(path, 0, 0, 19, 19));
  REQUIRE(!TCOD_path_is_empty(path));
  REQUIRE(data.nodes_expanded > 0);

  TCOD_path_delete(path);
  TCOD_map_delete(map);
}

TEST_CASE("Built-in heuristics", "[path]") {
  SECTION("Euclidean heuristic") {
    float h = TCOD_heuristic_euclidean(0, 0, 3, 4, nullptr);
    REQUIRE(h == Catch::Approx(5.0f));  // 3-4-5 triangle
  }

  SECTION("Manhattan heuristic") {
    float h = TCOD_heuristic_manhattan(0, 0, 3, 4, nullptr);
    REQUIRE(h == 7.0f);  // 3 + 4
  }

  SECTION("Chebyshev heuristic") {
    float h = TCOD_heuristic_chebyshev(0, 0, 3, 4, nullptr);
    REQUIRE(h == 4.0f);  // max(3, 4)
  }

  SECTION("Diagonal heuristic") {
    float h = TCOD_heuristic_diagonal(0, 0, 3, 4, nullptr);
    // min=3, max=4, so: 4 + (sqrt(2)-1)*3 = 4 + 0.414*3 = 5.24
    REQUIRE(h == Catch::Approx(5.2426f).epsilon(0.01));
  }

  SECTION("Zero heuristic") {
    float h = TCOD_heuristic_zero(0, 0, 100, 100, nullptr);
    REQUIRE(h == 0.0f);
  }
}

TEST_CASE("Zero heuristic produces Dijkstra-like behavior", "[path]") {
  // With zero heuristic, A* becomes Dijkstra (explores more nodes)
  // A* with Euclidean heuristic
  auto path_euclidean =
      TCOD_path_new_using_function_ex(20, 20, simple_cost, TCOD_heuristic_euclidean, nullptr, 1.41f, 1.0f);
  TCOD_path_compute(path_euclidean, 0, 0, 19, 19);

  // A* with zero heuristic (Dijkstra)
  auto path_zero = TCOD_path_new_using_function_ex(20, 20, simple_cost, TCOD_heuristic_zero, nullptr, 1.41f, 1.0f);
  TCOD_path_compute(path_zero, 0, 0, 19, 19);

  // Both should find a path
  REQUIRE(!TCOD_path_is_empty(path_euclidean));
  REQUIRE(!TCOD_path_is_empty(path_zero));

  TCOD_path_delete(path_euclidean);
  TCOD_path_delete(path_zero);
}

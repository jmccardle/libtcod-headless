/**
 * Tests for Dijkstra pathfinding with multi-goal support.
 *
 * These tests verify the new TCOD_dijkstra_compute_multi(),
 * TCOD_dijkstra_compute_masked(), TCOD_dijkstra_invert(),
 * and TCOD_dijkstra_get_descent() functions.
 */

#include <libtcod/fov.h>
#include <libtcod/path.h>

#include <catch2/catch_all.hpp>
#include <cmath>

// Simple uniform cost function for testing
static float uniform_cost(int xFrom, int yFrom, int xTo, int yTo, void* userData) {
  (void)xFrom;
  (void)yFrom;
  (void)xTo;
  (void)yTo;
  (void)userData;
  return 1.0f;
}

TEST_CASE("Dijkstra single-goal matches original behavior", "[dijkstra]") {
  auto map = TCOD_map_new(20, 20);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 20; y++) {
    for (int x = 0; x < 20; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  // Create two Dijkstra objects
  auto dijkstra_single = TCOD_dijkstra_new(map, 1.41f);
  auto dijkstra_multi = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra_single != nullptr);
  REQUIRE(dijkstra_multi != nullptr);

  // Compute with single root using original function
  TCOD_dijkstra_compute(dijkstra_single, 10, 10);

  // Compute with single root using multi function
  int roots_x[] = {10};
  int roots_y[] = {10};
  TCOD_dijkstra_compute_multi(dijkstra_multi, 1, roots_x, roots_y);

  // Compare distances at various points
  for (int y = 0; y < 20; y++) {
    for (int x = 0; x < 20; x++) {
      float dist_single = TCOD_dijkstra_get_distance(dijkstra_single, x, y);
      float dist_multi = TCOD_dijkstra_get_distance(dijkstra_multi, x, y);
      REQUIRE(dist_single == Catch::Approx(dist_multi));
    }
  }

  TCOD_dijkstra_delete(dijkstra_single);
  TCOD_dijkstra_delete(dijkstra_multi);
  TCOD_map_delete(map);
}

TEST_CASE("Dijkstra multi-goal: nearest goal wins", "[dijkstra]") {
  auto map = TCOD_map_new(20, 20);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 20; y++) {
    for (int x = 0; x < 20; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  auto dijkstra = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra != nullptr);

  // Two goals: (5,5) and (15,15)
  int roots_x[] = {5, 15};
  int roots_y[] = {5, 15};
  TCOD_dijkstra_compute_multi(dijkstra, 2, roots_x, roots_y);

  // Point (3,3) should be closer to (5,5)
  float dist_near = TCOD_dijkstra_get_distance(dijkstra, 3, 3);
  // Point (17,17) should be closer to (15,15)
  float dist_far = TCOD_dijkstra_get_distance(dijkstra, 17, 17);

  // Both should have similar small distances (about 2-3 tiles away)
  REQUIRE(dist_near == Catch::Approx(dist_far).margin(1.0f));
  REQUIRE(dist_near < 5.0f);
  REQUIRE(dist_far < 5.0f);

  // Middle point (10,10) should have some distance
  float dist_middle = TCOD_dijkstra_get_distance(dijkstra, 10, 10);
  REQUIRE(dist_middle > 0.0f);
  REQUIRE(dist_middle < 10.0f);

  TCOD_dijkstra_delete(dijkstra);
  TCOD_map_delete(map);
}

TEST_CASE("Dijkstra compute_masked", "[dijkstra]") {
  auto map = TCOD_map_new(10, 10);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  auto dijkstra = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra != nullptr);

  // Create mask with goals at corners
  uint8_t mask[100] = {0};
  mask[0] = 1;  // (0,0)
  mask[9] = 1;  // (9,0)
  mask[90] = 1;  // (0,9)
  mask[99] = 1;  // (9,9)

  TCOD_dijkstra_compute_masked(dijkstra, mask);

  // Center (5,5) should have distance to nearest corner
  float dist_center = TCOD_dijkstra_get_distance(dijkstra, 5, 5);
  // Distance should be about 7 tiles (diagonal to corner)
  REQUIRE(dist_center > 5.0f);
  REQUIRE(dist_center < 10.0f);

  // Corners should have distance 0
  REQUIRE(TCOD_dijkstra_get_distance(dijkstra, 0, 0) == 0.0f);
  REQUIRE(TCOD_dijkstra_get_distance(dijkstra, 9, 0) == 0.0f);
  REQUIRE(TCOD_dijkstra_get_distance(dijkstra, 0, 9) == 0.0f);
  REQUIRE(TCOD_dijkstra_get_distance(dijkstra, 9, 9) == 0.0f);

  TCOD_dijkstra_delete(dijkstra);
  TCOD_map_delete(map);
}

TEST_CASE("Dijkstra with function callback", "[dijkstra]") {
  auto dijkstra = TCOD_dijkstra_new_using_function(20, 20, uniform_cost, nullptr, 1.41f);
  REQUIRE(dijkstra != nullptr);

  // Test multi-goal with function-based Dijkstra
  int roots_x[] = {5, 15};
  int roots_y[] = {5, 15};
  TCOD_dijkstra_compute_multi(dijkstra, 2, roots_x, roots_y);

  // Middle point should have distance to nearest goal
  float dist = TCOD_dijkstra_get_distance(dijkstra, 10, 10);
  REQUIRE(dist > 0.0f);
  REQUIRE(dist < 15.0f);

  TCOD_dijkstra_delete(dijkstra);
}

TEST_CASE("Dijkstra multi-goal with overlapping roots", "[dijkstra]") {
  auto map = TCOD_map_new(10, 10);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  auto dijkstra = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra != nullptr);

  // Duplicate roots should be handled
  int roots_x[] = {5, 5, 5};
  int roots_y[] = {5, 5, 5};
  TCOD_dijkstra_compute_multi(dijkstra, 3, roots_x, roots_y);

  // Should work correctly with single effective root
  REQUIRE(TCOD_dijkstra_get_distance(dijkstra, 5, 5) == 0.0f);
  REQUIRE(TCOD_dijkstra_get_distance(dijkstra, 0, 0) > 0.0f);

  TCOD_dijkstra_delete(dijkstra);
  TCOD_map_delete(map);
}

TEST_CASE("Dijkstra invert creates flee map", "[dijkstra]") {
  auto map = TCOD_map_new(10, 10);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  auto dijkstra = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra != nullptr);

  // Single goal at center
  TCOD_dijkstra_compute(dijkstra, 5, 5);

  // Before invert: center has distance 0, corners have max distance
  float center_before = TCOD_dijkstra_get_distance(dijkstra, 5, 5);
  float corner_before = TCOD_dijkstra_get_distance(dijkstra, 0, 0);
  REQUIRE(center_before == 0.0f);
  REQUIRE(corner_before > center_before);

  // Invert the map
  TCOD_dijkstra_invert(dijkstra);

  // After invert: center has max distance, corners have distance 0
  float center_after = TCOD_dijkstra_get_distance(dijkstra, 5, 5);
  float corner_after = TCOD_dijkstra_get_distance(dijkstra, 0, 0);
  REQUIRE(center_after > corner_after);
  REQUIRE(corner_after < center_before + 1.0f);  // Corner should now be low

  TCOD_dijkstra_delete(dijkstra);
  TCOD_map_delete(map);
}

TEST_CASE("Dijkstra get_descent follows gradient", "[dijkstra]") {
  auto map = TCOD_map_new(10, 10);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  auto dijkstra = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra != nullptr);

  // Goal at (0,0)
  TCOD_dijkstra_compute(dijkstra, 0, 0);

  // Start at (5,5) and follow gradient
  int x = 5, y = 5;
  int out_x, out_y;
  int steps = 0;
  const int max_steps = 20;

  while (TCOD_dijkstra_get_descent(dijkstra, x, y, &out_x, &out_y) && steps < max_steps) {
    // Distance should decrease
    float old_dist = TCOD_dijkstra_get_distance(dijkstra, x, y);
    float new_dist = TCOD_dijkstra_get_distance(dijkstra, out_x, out_y);
    REQUIRE(new_dist < old_dist);

    x = out_x;
    y = out_y;
    steps++;
  }

  // Should have reached goal (0,0)
  REQUIRE(x == 0);
  REQUIRE(y == 0);

  TCOD_dijkstra_delete(dijkstra);
  TCOD_map_delete(map);
}

TEST_CASE("Dijkstra get_descent at goal returns false", "[dijkstra]") {
  auto map = TCOD_map_new(10, 10);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  auto dijkstra = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra != nullptr);

  TCOD_dijkstra_compute(dijkstra, 5, 5);

  // At goal, descent should return false
  int out_x, out_y;
  REQUIRE_FALSE(TCOD_dijkstra_get_descent(dijkstra, 5, 5, &out_x, &out_y));

  TCOD_dijkstra_delete(dijkstra);
  TCOD_map_delete(map);
}

TEST_CASE("Dijkstra flee behavior with inverted multi-goal", "[dijkstra]") {
  // Simulate a monster fleeing from multiple threats
  auto map = TCOD_map_new(20, 20);
  REQUIRE(map != nullptr);

  for (int y = 0; y < 20; y++) {
    for (int x = 0; x < 20; x++) {
      TCOD_map_set_properties(map, x, y, true, true);
    }
  }

  auto dijkstra = TCOD_dijkstra_new(map, 1.41f);
  REQUIRE(dijkstra != nullptr);

  // Two threats at (5,5) and (15,5)
  int threats_x[] = {5, 15};
  int threats_y[] = {5, 5};
  TCOD_dijkstra_compute_multi(dijkstra, 2, threats_x, threats_y);

  // Invert to create flee map
  TCOD_dijkstra_invert(dijkstra);

  // Monster at (10,5) - between the threats
  int x = 10, y = 5;
  int out_x, out_y;

  // Following gradient descent should move away from both threats
  if (TCOD_dijkstra_get_descent(dijkstra, x, y, &out_x, &out_y)) {
    // New position should be further from center line y=5
    // (fleeing up or down, not towards either threat)
    REQUIRE(out_y != 5);  // Should move away vertically
  }

  TCOD_dijkstra_delete(dijkstra);
  TCOD_map_delete(map);
}

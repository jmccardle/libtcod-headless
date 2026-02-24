/*
 * Copyright (c) 2026 libtcod contributors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file monastery_map.h
 * @brief Demonstration map for pathfinding heuristic comparison.
 *
 * The monastery map demonstrates where Euclidean heuristic wastes effort:
 *
 *   - Library (start, upper-left)
 *   - Chapel (goal, upper-right)
 *   - Long Corridor connects them at the top (optimal ~50 step path)
 *   - Great Hall is a large room below that ALSO connects to Chapel
 *     via a much longer route (~100 steps)
 *
 * Euclidean distance from Library to Great Hall to Chapel appears shorter
 * because the Great Hall is geometrically "closer" to the Chapel. A* with
 * Euclidean heuristic will explore the entire Great Hall before realizing
 * the corridor path is better.
 *
 * Waypoint-informed heuristic knows the room connectivity graph and
 * immediately guides search toward the corridor without wasting nodes
 * on the Great Hall.
 */
#pragma once
#ifndef MONASTERY_MAP_H_
#define MONASTERY_MAP_H_

#include <cmath>

// Map dimensions
static constexpr int MAP_WIDTH = 60;
static constexpr int MAP_HEIGHT = 30;

// Special positions
static constexpr int START_X = 5;
static constexpr int START_Y = 5;
static constexpr int GOAL_X = 54;
static constexpr int GOAL_Y = 5;

// Treasure positions for multi-goal Dijkstra demo
static constexpr int NUM_TREASURES = 4;
static constexpr int TREASURE_X[NUM_TREASURES] = {5, 54, 30, 54};
static constexpr int TREASURE_Y[NUM_TREASURES] = {25, 25, 20, 15};

// Threat positions for flee map demo (player + allies chasing monster)
static constexpr int NUM_THREATS = 3;
static constexpr int THREAT_X[NUM_THREATS] = {5, 8, 5};
static constexpr int THREAT_Y[NUM_THREATS] = {5, 5, 8};

// Room identifiers for waypoint graph
enum class Room {
  NONE = 0,
  LIBRARY = 1,
  CORRIDOR = 2,
  CHAPEL = 3,
  STAIRWELL = 4,
  GREAT_HALL = 5,
  CLOISTER = 6,
};

// Waypoint positions (center of each room/junction)
static constexpr int NUM_WAYPOINTS = 6;
struct Waypoint {
  int x, y;
  Room room;
  const char* name;
};

static constexpr Waypoint WAYPOINTS[NUM_WAYPOINTS] = {
    {5, 5, Room::LIBRARY, "Library"},
    {30, 5, Room::CORRIDOR, "Long Corridor"},
    {54, 5, Room::CHAPEL, "Chapel"},
    {5, 12, Room::STAIRWELL, "Stairwell"},
    {30, 20, Room::GREAT_HALL, "Great Hall"},
    {54, 20, Room::CLOISTER, "Cloister"},
};

// Waypoint graph edges (bidirectional connections with walking distances)
// These are the ACTUAL walking distances, not Euclidean
struct WaypointEdge {
  int from, to;
  float distance;
};

static constexpr int NUM_EDGES = 6;
static constexpr WaypointEdge EDGES[NUM_EDGES] = {
    {0, 1, 25.0f},  // Library -> Corridor (direct, optimal)
    {1, 2, 24.0f},  // Corridor -> Chapel (direct, optimal)
    {0, 3, 7.0f},  // Library -> Stairwell (going down)
    {3, 4, 32.0f},  // Stairwell -> Great Hall (down and across)
    {4, 5, 24.0f},  // Great Hall -> Cloister (across)
    {5, 2, 15.0f},  // Cloister -> Chapel (going up)
};

// Path comparison:
// Optimal: Library(0) -> Corridor(1) -> Chapel(2) = 25 + 24 = 49 steps
// Suboptimal: Library(0) -> Stairwell(3) -> Great Hall(4) -> Cloister(5) -> Chapel(2)
//           = 7 + 32 + 24 + 15 = 78 steps
//
// But Euclidean from Library(5,5) to Great Hall(30,20) to Chapel(54,5) = 28 + 28 = 56
// vs Library(5,5) to Corridor(30,5) to Chapel(54,5) = 25 + 24 = 49
// The Euclidean distances are similar, so A* explores both directions!

// clang-format off
/**
 * The monastery map layout (60x30).
 *
 * Legend:
 *   '#' = wall
 *   '.' = floor
 *   '+' = door (walkable)
 *
 * The key insight: Both paths reach the Chapel, but the corridor path is shorter.
 * Euclidean heuristic can't tell which is better and explores the Great Hall.
 */
static const char* MONASTERY_MAP[MAP_HEIGHT] = {
//   0         1         2         3         4         5
//   0123456789012345678901234567890123456789012345678901234567890
    "############################################################", // 0
    "#.........#................................................#", // 1
    "#.........#................................................#", // 2
    "#.........#................................................#", // 3
    "#.LIBRARY.+..........LONG.CORRIDOR........................#", // 4
    "#.........#................................................#", // 5
    "#.........#................................................#", // 6
    "#.........#...........................................CHAPEL", // 7
    "#.........#................................................#", // 8
    "#####+#####................................................#", // 9
    "#.........#............................................#####", // 10
    "#STAIRWELL#............................................#...#", // 11
    "#.........#............................................#...#", // 12
    "#.........#............................................#...#", // 13
    "#####+#######################################+#########+...#", // 14
    "#..................................................#...#...#", // 15
    "#..................................................#...#...#", // 16
    "#..................................................#...#...#", // 17
    "#.................GREAT.HALL.......................#...#...#", // 18
    "#..................................................#CLOISTER", // 19
    "#..................................................#...#...#", // 20
    "#..................................................#...#...#", // 21
    "#..................................................#...#...#", // 22
    "#..................................................#...#...#", // 23
    "#..................................................#...#...#", // 24
    "####################################################...#...#", // 25
    "#......................................................#...#", // 26
    "#......................................................#...#", // 27
    "#......................................................#...#", // 28
    "############################################################", // 29
};
// clang-format on

/**
 * Check if a map cell is a wall.
 */
inline bool is_wall(int x, int y) {
  if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return true;
  return MONASTERY_MAP[y][x] == '#';
}

/**
 * Check if a map cell is walkable.
 */
inline bool is_walkable(int x, int y) {
  if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return false;
  char c = MONASTERY_MAP[y][x];
  return c != '#';
}

/**
 * Get the character at a map position.
 */
inline char get_map_char(int x, int y) {
  if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return '#';
  return MONASTERY_MAP[y][x];
}

/**
 * Get the room ID for a given map position.
 * Used by the waypoint heuristic to determine which room a cell belongs to.
 */
inline Room get_room_at(int x, int y) {
  if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return Room::NONE;
  if (is_wall(x, y)) return Room::NONE;

  // Library (upper-left)
  if (x >= 1 && x <= 9 && y >= 1 && y <= 8) return Room::LIBRARY;

  // Stairwell (left side, below library)
  if (x >= 1 && x <= 9 && y >= 10 && y <= 13) return Room::STAIRWELL;

  // Long corridor (top, after library door)
  if (y >= 1 && y <= 9 && x >= 11 && x <= 57) return Room::CORRIDOR;

  // Chapel (upper-right corner area)
  if (x >= 50 && x <= 58 && y >= 7 && y <= 9) return Room::CHAPEL;
  if (x >= 55 && x <= 58 && y >= 10 && y <= 14) return Room::CHAPEL;

  // Great Hall (large center-bottom room)
  if (y >= 15 && y <= 24 && x >= 1 && x <= 50) return Room::GREAT_HALL;

  // Cloister (right side, connects Great Hall to Chapel)
  if (x >= 52 && x <= 58 && y >= 11 && y <= 28) return Room::CLOISTER;
  if (y >= 26 && y <= 28 && x >= 1 && x <= 58) return Room::CLOISTER;

  return Room::NONE;
}

/**
 * Find nearest waypoint to a given position.
 * Returns waypoint index, or -1 if position is not in any room.
 */
inline int find_nearest_waypoint(int x, int y) {
  Room room = get_room_at(x, y);
  if (room == Room::NONE) return -1;

  for (int i = 0; i < NUM_WAYPOINTS; ++i) {
    if (WAYPOINTS[i].room == room) return i;
  }
  return -1;
}

/**
 * Compute Euclidean distance between two points.
 */
inline float euclidean_distance(int x1, int y1, int x2, int y2) {
  float dx = static_cast<float>(x2 - x1);
  float dy = static_cast<float>(y2 - y1);
  return sqrtf(dx * dx + dy * dy);
}

#endif  // MONASTERY_MAP_H_

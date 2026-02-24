# Pathfinding Heuristic Demo

This demo showcases the power of **domain-knowledge heuristics** compared to naive geometric heuristics in A* pathfinding.

## The Key Insight

The real value of custom heuristics isn't geometric formulas (anyone can write `sqrt(dx*dx + dy*dy)`). The value is **injecting domain knowledge** into the search:

| Technique | What the Heuristic Uses | Benefit |
|-----------|-------------------------|---------|
| Waypoint graphs | Precomputed room connectivity | Skips dead-end rooms |
| Hierarchical | Coarse-grid Dijkstra distances | Captures map topology |
| True distance | Precomputed landmark distances | Near-perfect guidance |

All share a pattern: **precomputed data structures** that the heuristic queries.

## The Monastery Map

The demo uses a monastery map designed to expose Euclidean heuristic weaknesses:

```
    +---------+                                   +---------+
    | LIBRARY +---------LONG CORRIDOR------------+ CHAPEL  |
    |   @     |                                   |    X    |
    +----+----+                                   +---------+
         |
    +----+----+
    |STAIRWELL|
    +----+----+
         |
    +----+-------------------------------------------+
    |                    GREAT HALL                  |
    |   (Euclidean thinks this is progress!)        |
    +------------------------------------------------+
```

**Problem:** The Library-to-Chapel path through the corridor is ~50 steps. But Euclidean distance says going toward the Great Hall is "progress" because it's geometrically closer to the Chapel. A* with Euclidean heuristic explores the entire Great Hall before finding the corridor path.

**Solution:** A waypoint-informed heuristic knows the room connectivity graph. It immediately guides search toward the corridor without wasting nodes on the Great Hall.

## Demo Modes

### 1. Euclidean A* (Baseline)
Standard A* with Euclidean distance heuristic. Watch it explore the Great Hall.

**Metrics:** ~800+ nodes expanded

### 2. Waypoint A*
A* with precomputed room connectivity graph. The heuristic uses:
```
h(x,y) = dist_to_waypoint + graph_distance + dist_from_goal_waypoint
```

**Metrics:** ~100 nodes expanded (8x improvement!)

### 3. Hierarchical A*
A* with minimap (4:1 scale) Dijkstra distances. No manual room annotation needed - the minimap naturally captures topology.

**Metrics:** Between Euclidean and Waypoint

### 4. Multi-Goal Dijkstra
Demonstrates `TCOD_dijkstra_compute_multi()` with multiple treasure positions. The gradient shows "nearest treasure" from any cell.

### 5. Flee Map (Inverted Dijkstra)
Demonstrates `TCOD_dijkstra_invert()` for AI flee behavior. Multiple threats (player + allies) create a danger map. Inverting it creates a safety map - the monster flees toward safe areas (far from all threats), not just away from the nearest one.

## Controls

| Key | Action |
|-----|--------|
| `1` | Euclidean A* mode |
| `2` | Waypoint A* mode |
| `3` | Hierarchical A* mode |
| `4` | Multi-Goal Dijkstra mode |
| `5` | Flee Map mode |
| `W` | Toggle waypoint graph overlay |
| `H` | Toggle hierarchical minimap overlay |
| `R` | Reset/recompute |
| `Q` | Quit |
| `Alt+Enter` | Toggle fullscreen |

## Building

From the libtcod build directory:

```bash
cmake .. -DLIBTCOD_SAMPLES=ON
make pathfinding
./pathfinding
```

## Technical Details

### Waypoint Graph Implementation

The waypoint graph stores:
1. Shortest paths between all room waypoints (Floyd-Warshall)
2. For each cell, distance to its room's waypoint
3. Goal waypoint and distance from goal to its waypoint

The heuristic is admissible because it uses actual walking distances through the room graph.

### Hierarchical Implementation

The minimap is created by:
1. Reduce map to 4:1 scale (15x8 from 60x30)
2. A mini-cell is walkable if ANY source cell is walkable
3. Compute Dijkstra on minimap from goal region
4. Heuristic = minimap_distance * 4

This is admissible because the minimap can only have shorter paths (fewer obstacles).

### Flee Map Implementation

Brogue-inspired technique:
1. Compute Dijkstra from all threat positions
2. Invert distances: `new_dist = max_dist - old_dist`
3. Use gradient descent to flee

The monster doesn't just run from the nearest threat - it finds the globally safest position considering ALL threats.

## API Functions Used

This demo showcases the new pathfinding API functions:

### A* Heuristics
- `TCOD_path_new_using_function_ex()` - Create pathfinder with custom heuristic
- `TCOD_path_set_heuristic()` - Change heuristic on existing pathfinder
- `TCOD_heuristic_euclidean()` - Built-in Euclidean distance
- Custom `TCOD_heuristic_func_t` callbacks

### Dijkstra Extensions
- `TCOD_dijkstra_compute_multi()` - Multiple goal positions
- `TCOD_dijkstra_compute_masked()` - Goals from bitmap mask
- `TCOD_dijkstra_invert()` - Create flee/safety maps
- `TCOD_dijkstra_get_descent()` - Gradient descent for movement

## References

- [Brogue's Dijkstra Maps](https://www.roguebasin.com/index.php/The_Incredible_Power_of_Dijkstra_Maps) - Brian Walker's AI techniques
- [Hierarchical Pathfinding](http://aigamedev.com/open/review/near-optimal-hierarchical-path-finding/) - HPA* and related algorithms

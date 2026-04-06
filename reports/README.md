# Libtcod Issue Research Reports

This directory contains comprehensive research reports for open libtcod issues. Each report synthesizes the issue thread discussion, current codebase analysis, modern best practices research, and recommended implementation approaches.

**Generated**: January 24, 2026
**Codebase**: libtcod-headless (McRogueFace fork)
**Branch**: feature/heightmap-convolution

## Report Index

| Issue | Title | Priority | Status | Report |
|-------|-------|----------|--------|--------|
| [#5](https://github.com/libtcod/libtcod/issues/5) | Rewrite Dijkstra Implementation | High | Open | [issue_005_dijkstra_multigoal.md](issue_005_dijkstra_multigoal.md) |
| [#60](https://github.com/libtcod/libtcod/issues/60) | A* Custom Heuristic Function | Medium | Open | [issue_060_custom_heuristic.md](issue_060_custom_heuristic.md) |
| [#71](https://github.com/libtcod/libtcod/issues/71) | Refactoring TCOD's FOV Algorithms | Low | Open (tracking) | [issue_071_fov_refactoring.md](issue_071_fov_refactoring.md) |
| [#92](https://github.com/libtcod/libtcod/issues/92) | Line Handling Improvements | Medium | Partial | [issue_092_line_handling.md](issue_092_line_handling.md) |
| [#110](https://github.com/libtcod/libtcod/issues/110) | fov_permissive2 - Bumps Usage | N/A | Resolved | [issue_110_permissive_bumps.md](issue_110_permissive_bumps.md) |

## Summary of Findings

### Issue #5: Multi-Goal Dijkstra Maps (High Priority)

**Current State**: Single-goal only limitation prevents "Dijkstra map" patterns for AI.

**Recommended Action**: Add `TCOD_dijkstra_compute_multi()` and utility functions for safety maps, auto-exploration, and desire-driven AI. This is a high-value enhancement with broad utility.

**Key Files**: `src/libtcod/path_c.c`, `src/libtcod/path.h`

---

### Issue #60: A* Custom Heuristic Function (Medium Priority)

**Current State**: Hardcoded Euclidean heuristic at line 406 of `path_c.c`.

**Recommended Action**: Add `TCOD_path_new_using_function_ex()` with heuristic callback parameter. Minimal change (~20 lines), maintains backward compatibility.

**Key Files**: `src/libtcod/path_c.c`, `src/libtcod/path.h`

---

### Issue #71: FOV Refactoring (Low Priority - Tracking)

**Current State**: Six algorithms implemented, each in separate file. Architecture is functional but could be modernized.

**Recommended Action**: Incremental refactoring:
1. Create common infrastructure (`fov_common.h`)
2. Separate post-processing from algorithms
3. Add array-based API alongside `TCOD_Map`

**Key Files**: `src/libtcod/fov_c.c`, `src/libtcod/fov_*.c`

---

### Issue #92: Line Handling Improvements (Partially Complete)

**Current State**: C++ `tcod::BresenhamLine` class complete with iterators, bidirectional traversal, and range control. C API lacks array output and userdata callbacks.

**Recommended Action**: Add `TCOD_line_to_array()` and `TCOD_line_ex()` with userdata parameter.

**Key Files**: `src/libtcod/bresenham_c.c`, `src/libtcod/bresenham.h`

---

### Issue #110: Permissive FOV Bumps (Resolved)

**Current State**: Question answered - `ViewBumpContainer` is an arena allocator, not a traditional collection.

**Recommended Action**: None. Design is correct and intentional.

**Key Files**: `src/libtcod/fov_permissive2.c`

## Key Resources

### Pathfinding
- [Stanford A* Heuristics](http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html)
- [RogueBasin: Dijkstra Maps](https://www.roguebasin.com/index.php/The_Incredible_Power_of_Dijkstra_Maps)
- [A* Search Algorithm](https://www.geeksforgeeks.org/dsa/a-search-algorithm/)

### Field of View
- [Adam Millington: Roguelike Vision Algorithms](http://www.adammil.net/blog/v125_Roguelike_Vision_Algorithms.html)
- [Albert Ford: Symmetric Shadowcasting](https://www.albertford.com/shadowcasting/)
- [RogueBasin: FOV Algorithms](https://www.roguebasin.com/index.php/Field_of_Vision)

### Line Drawing
- [libtcod BresenhamLine Documentation](https://libtcod.readthedocs.io/en/2.0.0/library/classtcod_1_1_bresenham_line.html)
- [RogueBasin: Bresenham's Algorithm](https://www.roguebasin.com/index.php/Bresenham's_Line_Algorithm)

## Implementation Notes

### Backward Compatibility

All recommended changes maintain backward compatibility:
- New functions added alongside existing ones
- Default behavior preserved when new parameters are omitted
- No existing function signatures modified

### Code Style

libtcod follows these patterns:
- C99 for core implementation
- C++ wrappers with modern features (C++17)
- Functions prefixed with `TCOD_`
- Structs/types with `TCOD_` prefix and `_t` suffix for typedef handles
- Error handling via `TCOD_Error` return values

### Testing

Each enhancement should include:
- Unit tests for new functions
- Regression tests ensuring existing behavior unchanged
- Performance benchmarks where applicable

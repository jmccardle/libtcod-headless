# Issue #92: Line Handling Improvements

## Summary

**Issue**: [Line handling improvements #92](https://github.com/libtcod/libtcod/issues/92)
**Opened**: April 16, 2021 by HexDecimal
**Status**: Partially Complete
**Related PR**: [Add iterator implementation to BresenhamLine #93](https://github.com/libtcod/libtcod/pull/93)

## Issue Description

HexDecimal outlined improvements for the Bresenham line drawing module, focusing on modernizing both C and C++ APIs:

### Original Checklist

| Item | Description | Status |
|------|-------------|--------|
| C array output | Function that outputs full array of line indexes to buffer | **Not Done** |
| C++ forward iterator with range support | Iterator for range-based for loops | **Done** |
| C++ iterator with custom ranges | Adjustable start/end points | **Done** |
| C++ bidirectional iterator | Forward and backward iteration | **Done** |
| C++ arbitrary iteration | Random access to any point on line | **Done** |

Additional goals mentioned:
- Move line drawing code inline (C99 inline functions)
- C callbacks need userdata parameter
- C++ should support lambdas as callbacks

## Current Implementation Analysis

### C Implementation (`bresenham_c.c`)

**Thread-safe functions:**
```c
void TCOD_line_init_mt(int xFrom, int yFrom, int xTo, int yTo, TCOD_bresenham_data_t* data);
bool TCOD_line_step_mt(int* xCur, int* yCur, TCOD_bresenham_data_t* data);
```

**Callback-based:**
```c
typedef bool (*TCOD_line_listener_t)(int x, int y);
bool TCOD_line(int xo, int yo, int xd, int yd, TCOD_line_listener_t listener);
```

**Deprecated (non-reentrant):**
```c
void TCOD_line_init(int xFrom, int yFrom, int xTo, int yTo);  // Uses static variable
bool TCOD_line_step(int* xCur, int* yCur);  // Uses static variable
```

### C++ Implementation (`bresenham.hpp`)

The `tcod::BresenhamLine` class (added in v1.17) provides modern C++ iteration:

```cpp
class BresenhamLine {
public:
    using Point2 = std::array<int, 2>;
    using iterator_category = std::random_access_iterator_tag;

    explicit BresenhamLine(Point2 begin, Point2 end) noexcept;

    // Random access
    value_type operator[](int index) noexcept;
    value_type operator*() noexcept;

    // Bidirectional iteration
    BresenhamLine& operator++() noexcept;
    BresenhamLine& operator--() noexcept;

    // Range modification
    BresenhamLine adjust_range(int shift_begin, int shift_end) const noexcept;
    BresenhamLine without_start() const noexcept;
    BresenhamLine without_end() const noexcept;
    BresenhamLine without_endpoints() const noexcept;

    // Range-based for loop support
    BresenhamLine begin() const noexcept;
    BresenhamLine end() const noexcept;
};
```

**Usage Example:**
```cpp
for (auto&& [x, y] : tcod::BresenhamLine({0, 0}, {10, 10})) {
    // Process each point
}

// Without endpoints
for (auto&& [x, y] : tcod::BresenhamLine(from, to).without_endpoints()) {
    // Excludes both from and to
}
```

## Remaining Work: C Array Output

### Proposed API

From HexDecimal's comment:
```c
int length = TCOD_bresenham(start_x, start_y, end_x, end_y, 0, NULL);  // Query length
int* indexes = malloc(sizeof(*indexes) * length * 2);
TCOD_bresenham(0, 0, 4, 4, length, indexes);  // Fill buffer
```

### Recommended Implementation

Add to `bresenham.h`:
```c
/**
 * @brief Compute a Bresenham line and optionally output to buffer.
 *
 * @param x1 Starting X coordinate.
 * @param y1 Starting Y coordinate.
 * @param x2 Ending X coordinate.
 * @param y2 Ending Y coordinate.
 * @param buffer_size Size of the output buffer (number of points, not bytes).
 * @param out_x Output buffer for X coordinates, or NULL to only compute length.
 * @param out_y Output buffer for Y coordinates, or NULL to only compute length.
 * @return The number of points in the line (including both endpoints).
 *
 * If out_x and out_y are NULL, only returns the line length.
 * If buffer_size is less than the line length, only fills buffer_size points.
 */
int TCOD_line_to_array(
    int x1, int y1,
    int x2, int y2,
    int buffer_size,
    int* out_x,
    int* out_y
);

/**
 * @brief Compute a Bresenham line to an interleaved coordinate buffer.
 *
 * @param x1 Starting X coordinate.
 * @param y1 Starting Y coordinate.
 * @param x2 Ending X coordinate.
 * @param y2 Ending Y coordinate.
 * @param buffer_size Number of coordinate pairs the buffer can hold.
 * @param out Output buffer for [x,y,x,y,...] pairs, or NULL to only compute length.
 * @return The number of points in the line.
 */
int TCOD_line_to_array_interleaved(
    int x1, int y1,
    int x2, int y2,
    int buffer_size,
    int* out
);
```

### Implementation (`bresenham_c.c`)

```c
int TCOD_line_to_array(int x1, int y1, int x2, int y2, int buffer_size, int* out_x, int* out_y) {
    TCOD_bresenham_data_t data;
    TCOD_line_init_mt(x1, y1, x2, y2, &data);

    int count = 0;
    int x = x1, y = y1;

    // Include starting point
    if (out_x && out_y && count < buffer_size) {
        out_x[count] = x;
        out_y[count] = y;
    }
    count++;

    // Iterate through remaining points
    while (!TCOD_line_step_mt(&x, &y, &data)) {
        if (out_x && out_y && count < buffer_size) {
            out_x[count] = x;
            out_y[count] = y;
        }
        count++;
    }

    return count;
}

int TCOD_line_to_array_interleaved(int x1, int y1, int x2, int y2, int buffer_size, int* out) {
    TCOD_bresenham_data_t data;
    TCOD_line_init_mt(x1, y1, x2, y2, &data);

    int count = 0;
    int x = x1, y = y1;

    if (out && count < buffer_size) {
        out[count * 2] = x;
        out[count * 2 + 1] = y;
    }
    count++;

    while (!TCOD_line_step_mt(&x, &y, &data)) {
        if (out && count < buffer_size) {
            out[count * 2] = x;
            out[count * 2 + 1] = y;
        }
        count++;
    }

    return count;
}
```

## C Callback Improvements

### Current Limitation

The current callback signature lacks userdata:
```c
typedef bool (*TCOD_line_listener_t)(int x, int y);
```

### Proposed Enhancement

```c
// New callback with userdata
typedef bool (*TCOD_line_listener_ex_t)(int x, int y, void* userdata);

// New function using enhanced callback
bool TCOD_line_ex(
    int xo, int yo,
    int xd, int yd,
    TCOD_line_listener_ex_t listener,
    void* userdata
);
```

This follows the pattern used elsewhere in libtcod (e.g., `TCOD_path_func_t`).

## Inline Functions (C99)

HexDecimal mentioned moving code inline for low-level tasks. However:

1. **Current state**: Core Bresenham logic is already relatively small
2. **Consideration**: The C++ `BresenhamLine` effectively provides inline iteration
3. **Risk**: Inline functions in headers increase compilation dependencies

**Recommendation**: Keep C implementation in `.c` file, rely on compiler optimizations (LTO). The C++ class already provides zero-overhead abstraction for performance-critical code.

## C++ Lambda Support

The legacy `TCODLineListener` class requires inheritance:
```cpp
class TCODLineListener {
public:
    virtual bool putPoint(int x, int y) = 0;
};
```

### Modern Alternative

The `tcod::BresenhamLine` iterator approach eliminates the need for callbacks entirely:
```cpp
// Instead of callbacks, use range-based iteration
for (auto&& [x, y] : tcod::BresenhamLine(from, to)) {
    if (!processPoint(x, y)) break;
}
```

For callback-style usage, a wrapper function could be added:
```cpp
namespace tcod {
template<typename Func>
bool line_foreach(Point2 from, Point2 to, Func&& callback) {
    for (auto&& [x, y] : BresenhamLine(from, to)) {
        if (!callback(x, y)) return false;
    }
    return true;
}
}

// Usage:
tcod::line_foreach({0, 0}, {10, 10}, [&](int x, int y) {
    return map.is_valid(x, y);
});
```

## Implementation Status Summary

| Feature | C | C++ | Notes |
|---------|---|-----|-------|
| Thread-safe iteration | Yes | Yes | `_mt` functions, BresenhamLine class |
| Range-based for | N/A | Yes | BresenhamLine iterator |
| Bidirectional | No | Yes | `operator--` |
| Random access | No | Yes | `operator[]` |
| Endpoint control | No | Yes | `without_start/end/endpoints` |
| Array output | **No** | Use iterator | Proposed `TCOD_line_to_array` |
| Userdata callback | **No** | Use lambda | Proposed `TCOD_line_ex` |

## Testing Strategy

1. **Array output tests**: Verify output matches iterator traversal
2. **Boundary tests**: Lines of length 0, 1, horizontal/vertical/diagonal
3. **Large coordinate tests**: Ensure no overflow with large coordinates
4. **Userdata callback tests**: Verify state is correctly passed

## References

- [libtcod BresenhamLine Documentation](https://libtcod.readthedocs.io/en/2.0.0/library/classtcod_1_1_bresenham_line.html)
- [libtcod Line Drawing Toolkit](https://libtcod.readthedocs.io/en/latest/base-toolkits/line_drawing.html)
- [libtcod/bresenham.hpp](https://github.com/libtcod/libtcod/blob/main/src/libtcod/bresenham.hpp)
- [RogueBasin: Bresenham's Line Algorithm](https://www.roguebasin.com/index.php/Bresenham's_Line_Algorithm)
- [GeeksforGeeks: Bresenham's Line Generation Algorithm](https://www.geeksforgeeks.org/dsa/bresenhams-line-generation-algorithm/)

## Conclusion

The C++ portion of this issue is complete with the modern `tcod::BresenhamLine` class providing:
- Forward and bidirectional iteration
- Random access to any point
- Range-based for loop support
- Endpoint control (`without_start`, `without_end`, `without_endpoints`)

Remaining C work:
1. **High Priority**: Add `TCOD_line_to_array` for buffer output
2. **Medium Priority**: Add `TCOD_line_ex` with userdata callback
3. **Low Priority**: Consider inline functions (may not provide significant benefit)

The C++ iterator approach is the recommended path for new code, with C functions maintained for language bindings and legacy compatibility.

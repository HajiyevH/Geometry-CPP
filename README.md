 Polyline Proximity Check (C++)

This project provides C++ implementations to determine if two polylines (represented as sequences of connected line segments) come closer to each other than a specified distance threshold.

## Problem Statement

Given two polylines, `polyline1` and `polyline2`, represented as `std::vector<sPoint2D>`, determine if the minimum distance between any point on `polyline1` and any point on `polyline2` is less than a given `DISTANCE_THRESHOLD` (1.5f).

The core task is implemented in the function:
```cpp
bool arePolylinesCloserThanThreshold(
    const std::vector<sPoint2D>& polyline1,
    const std::vector<sPoint2D>& polyline2
);
```

**Input Example:**
```cpp
std::vector<sPoint2D> polyline1 { {2.0F, 3.0F}, {3.0F, 4.0F}, {2.0F, 6.0F} };
std::vector<sPoint2D> polyline2 { {5.0F, 6.0F}, {5.0F, 4.0F}, {7.0F, 4.0F}, {7.0F, 2.0F} };
// Expected Output: false (minimum distance is 2.0F)
```

**Data Structures:**
*   `sPoint2D`: Represents a 2D point with `float x` and `float y`.
*   `BoundingBox`: Represents an Axis-Aligned Bounding Box (AABB) with `minX`, `maxX`, `minY`, `maxY`.

## Implemented Approaches

The `distanceCheck.cpp` file contains three different approaches to solve this problem:

1.  **Brute Force (`arePolylinesCloserThanThresholdBrute`)**
    *   **Idea:** Compares every segment of `polyline1` against every segment of `polyline2`.
    *   **Method:** For each pair of segments (one from each polyline), it calculates the distance between the endpoints of one segment and the entirety of the other segment (4 point-to-segment checks per pair).

2.  **Bounding Box Optimization (`arePolylinesCloserThanThresholdBoundingBox`)**
    *   **Idea:** Reduces unnecessary distance calculations using bounding boxes.
    *   **Method:** Before performing the expensive point-to-segment distance checks for a pair of segments, it first checks if their bounding boxes (expanded by the `DISTANCE_THRESHOLD`) overlap using `boundingBoxesPotentiallyClose`. The detailed distance check is skipped if the expanded boxes do not overlap.

3.  **Spatial Grid (`arePolylinesCloserThanThreshold`)**
    *   **Idea:** Divides the 2D space containing the polylines into a uniform grid to quickly find potentially close segment pairs.
    *   **Method:**
        *   **Grid Setup:** Calculates the total bounding box for both polylines and divides it into grid cells of size `DISTANCE_THRESHOLD`.
        *   **Population:** Each segment from both polylines is inserted into a hash map (`grid`). The keys are cell coordinates, and the values are lists of segments whose bounding boxes overlap that cell.
        *   **Checking:** Iterates through each *populated* grid cell. For each cell, it checks for close segments:
            *   Between pairs of segments *within* the same cell (one from polyline1, one from polyline2).
            *   Between segments in the current cell and segments in adjacent *neighboring* cells (using `checkCellPairs`).
    *   This approach significantly reduces the number of segment pairs that need detailed distance checks, especially when polylines are large or sparse.

## Complexity Analysis

`N` = number of points in `polyline1`, `M` = number of points in `polyline2`.

| Algorithm                     | Worst Case Time | Average Case Time      | Notes                                                                 |
| :---------------------------- | :-------------- | :--------------------- | :-------------------------------------------------------------------- |
| Brute Force                   | `O(N*M)`        | `O(N*M)`               | Simple but inefficient for large polylines.                           |
| Bounding Box                  | `O(N*M)`        | Often faster than Brute | Performance depends on bounding box overlap; still `O(N*M)` worst case. |
| Spatial Grid                  | `O((N+M)^2)`    | `O(N+M)`               | Average case assumes good spatial distribution & hash map performance.  |
|                               |                 |                        | Worst case occurs if most segments cluster in few cells.              |

*   **Spatial Grid Notes:** The average case `O(N+M)` assumes segments are reasonably distributed, and hash map operations are `O(1)` on average. The implementation iterates through populated cells, improving efficiency over checking all possible grid cells.

## Further Improvements

1.  **Advanced Spatial Data Structures:**
    *   **Quadtrees/kD-Trees:** Adaptively partition space, potentially offering better performance than uniform grids for non-uniform data distributions. Build time is typically `O((N+M)log(N+M))`.
    *   **R-Trees:** Optimized for spatial objects with extent (like bounding boxes), efficient for finding overlapping regions.
3.  **Parallelization:**
    *   Brute-force and Bounding Box methods are highly parallelizable as segment pair checks are independent.
    *   Spatial Grid population and checking phases can be partially parallelized (requires careful handling of concurrent access).
```<!-- filepath: /Users/hajiaga/Desktop/Personal/Geometry-CPP/README.md -->

// Explorer.h
#ifndef EXPLORER_H
#define EXPLORER_H

#include "GridMap.h"
#include <string.h>
#include <stdint.h>

// A simple list to store a sequence of points
struct PointList {
  static const int MAX_SIZE = GRID_ROWS * GRID_COLS;
  Point data[MAX_SIZE];
  int   length;

  PointList() : length(0) {}

  bool empty() const { return length == 0; }
  void clear()       { length = 0; }

  // Adds a point to the list if theres space
  bool push(const Point& p) {
    if (length < MAX_SIZE) {
      data[length++] = p;
      return true;
    }
    return false;
  }

  // Access elements by index
  Point& operator[](int i) { return data[i]; }
  const Point& operator[](int i) const { return data[i]; }
};

class Explorer {
public:
  // Constructor takes a reference to a map
  explicit Explorer(GridMap& map);

  // Finds the nearest frontier cell a FREE cell next to UNKNOWN space
  Point findNearestFrontierCell(const Point& startCell) const;

  // Finds a path from start to goal using Bfs
  PointList planPathTo(const Point& startCell,
                       const Point& goalCell) const;

private:
  GridMap& gridMap_;  // Reference to the map

  // Direction offsets for moving up, down, left, and right
  static const int kDirections[4][2];

  // Constants and buffers used for BFS
  static const int kTotalCells = GRID_ROWS * GRID_COLS;
  static const int kVisitedBytes = (kTotalCells + 7) / 8;

  static Point pathQueue[kTotalCells];              // BFS queue
  static Point pathParent[GRID_ROWS][GRID_COLS];    // Stores where each cell came from
  static Point pathTemp[kTotalCells];               // Temporary storage for the path
  static uint8_t pathVisitedBits[kVisitedBytes];      // Bitfield for visited cells

  // Marks all cells as unvisited
  static inline void clearVisited() {
    memset(pathVisitedBits, 0, kVisitedBytes);
  }

  // Checks if a cell was visited
  static inline bool isVisited(int r, int c) {
    int idx = r * GRID_COLS + c;
    return pathVisitedBits[idx >> 3] & (1 << (idx & 7));
  }

  // Marks a cell as visited
  static inline void setVisited(int r, int c) {
    int idx = r * GRID_COLS + c;
    pathVisitedBits[idx >> 3] |= (1 << (idx & 7));
  }
};

#endif  // EXPLORER_H

// Explorer.h
#ifndef EXPLORER_H
#define EXPLORER_H

#include "GridMap.h"
#include <string.h>
#include <stdint.h>

// A tiny fixed‐capacity list to hold a sequence of Points.
struct PointList {
  static const int MAX_SIZE = GRID_ROWS * GRID_COLS;
  Point data[MAX_SIZE];
  int   length;
  PointList(): length(0) {}
  bool empty() const            { return length == 0; }
  void clear()                  { length = 0; }
  bool push(const Point& p) {
    if (length < MAX_SIZE) {
      data[length++] = p;
      return true;
    }
    return false;
  }
  Point& operator[](int i)       { return data[i]; }
  const Point& operator[](int i) const { return data[i]; }
};

class Explorer {
public:
  explicit Explorer(GridMap& map);

  // Scan‐based frontier finder—no extra big buffers
  Point findNearestFrontierCell(const Point& startCell) const;

  // BFS path through FREE cells, using a packed visited bitfield
  PointList planPathTo(const Point& startCell,
                       const Point& goalCell) const;

private:
  GridMap& gridMap_;

  // 4‐way neighbor offsets
  static const int kDirections[4][2];

  // For path BFS:
  static const int  kTotalCells    = GRID_ROWS * GRID_COLS;
  static const int  kVisitedBytes  = (kTotalCells + 7) / 8;
  static       Point  pathQueue[kTotalCells];
  static       Point  pathParent[GRID_ROWS][GRID_COLS];
  static       Point  pathTemp[kTotalCells];
  static       uint8_t pathVisitedBits[kVisitedBytes];

  // Helpers to set/test/clear visited bits
  static inline void    clearVisited()  { memset(pathVisitedBits, 0, kVisitedBytes); }
  static inline bool    isVisited(int r, int c) {
    int idx = r * GRID_COLS + c;
    return pathVisitedBits[idx >> 3] & (1 << (idx & 7));
  }
  static inline void    setVisited(int r, int c) {
    int idx = r * GRID_COLS + c;
    pathVisitedBits[idx >> 3] |= (1 << (idx & 7));
  }
};

#endif  // EXPLORER_H

// GridMap.h
#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <stdint.h>

// dimensions
static const int GRID_ROWS   = 9;
static const int GRID_COLS   = 19;
static const float CELL_SIZE = 10.0f;  // cm

// occupancy values
enum Cell : int8_t { UNKNOWN = -1, FREE = 0, OCCUPIED = 1 };

// simple row/col struct used throughout
struct Point {
  int8_t row;
  int8_t col;
};

class GridMap {
public:
  GridMap();
  void reset();

  // Convert world‐coords (cm) grid cell (row,col), clamped
  Point worldToGrid(float x, float y) const;

  void markFree(float x, float y);
  void markOcc (float x, float y);

  Cell getCell(int r, int c) const;

private:
  Cell grid[GRID_ROWS][GRID_COLS];
};

#endif  // GRIDMAP_H

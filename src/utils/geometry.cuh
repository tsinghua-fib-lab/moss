#ifndef SRC_UTILS_GEOMETRY_H_
#define SRC_UTILS_GEOMETRY_H_

#include <cuda.h>
#include "utils/macro.h"

// fp32 point/vector with basic operations
struct Point {
  float x, y, z;
  __host__ __device__ Point operator+(const Point& a) const {
    return {x + a.x, y + a.y, z + a.z};
  }
  __host__ __device__ Point operator-(const Point& a) const {
    return {x - a.x, y - a.y, z - a.z};
  }
  __host__ __device__ Point operator*(float a) const { return {x * a, y * a, z * a}; }
  __host__ __device__ Point operator/(float a) const { return {x / a, y / a, z / a}; }
  __host__ __device__ Point& operator*=(float a) {
    x *= a;
    y *= a;
    z *= a;
    return *this;
  }
  __host__ __device__ float Length() const { return hypot(x, y, z); }
  __host__ __device__ float SquareLength() const { return x * x + y * y + z * z; }
  // this * (1 - k) + a * k
  __host__ __device__ Point Blend(const Point& a, float k) const {
    return {x * (1 - k) + a.x * k, y * (1 - k) + a.y * k, z * (1 - k) + a.z * k};
  }
  // Move by a vector: this + a * k
  __host__ __device__ void Move(const Point& a, float k) {
    x += a.x * k;
    y += a.y * k;
    z += a.z * k;
  }
  // Dot product
  __host__ __device__ float Dot(const Point& a) const {
    return x * a.x + y * a.y + z * a.z;
  }
  // Cross2D product
  __host__ __device__ float Cross2D(const Point& a) const {
    return x * a.y - y * a.x;
  }
  // Angle with x-axis in radians
  __host__ __device__ float Dir() const { return atan2(y, x); }
  // Pitch
  __host__ __device__ float Pitch() const { return atan2(z, hypot(x, y)); }
};

struct PolylineDirection {
  float dir;
  float pitch;
};

// Get the accumulated lengths of a polyline
// The first element is always 0
// The output array should have at least size elements
void GetPolylineLengths2D(float* output, const Point* line, uint size);

// Get the directions of each segment in a polyline
// The output array should have at least size - 1 elements
void GetPolylineDirections(PolylineDirection* output, const Point* line, uint size);

// return -1 if x < -eps, 0 if -eps <= x <= eps, 1 if x > eps
// eps is 1e-6
__host__ __device__ int dcmp(float x);

// 用计算几何中的射线法判断点是否在任意多边形内部（不含边）
// 取水平向右的射线，计算与多边形边的交点数，根据结果的奇偶性判断是否在内部
// 请保证positions各点顺序给出，且第一点与最后一点相同
// use ray-casting algorithm to determine if a point is inside a polygon (excluding edges)
// cast a horizontal ray to the right, count the number of intersections with polygon edges
// determine if the point is inside based on the parity of the intersection count
// please ensure the order of positions and the first and last points are the same
__host__ __device__ bool InPolygon2D(const Point& a, const Point*, uint n);
#endif

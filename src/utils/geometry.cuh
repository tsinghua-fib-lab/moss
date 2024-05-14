#ifndef SRC_UTILS_GEOMETRY_H_
#define SRC_UTILS_GEOMETRY_H_

#include <cuda.h>
#include "utils/macro.h"

struct Point {
  float x, y;
  __host__ __device__ Point operator+(const Point& a) const {
    return {x + a.x, y + a.y};
  }
  __host__ __device__ Point operator-(const Point& a) const {
    return {x - a.x, y - a.y};
  }
  __host__ __device__ Point operator*(float a) const { return {x * a, y * a}; }
  __host__ __device__ Point operator/(float a) const { return {x / a, y / a}; }
  __host__ __device__ Point& operator*=(float a) {
    x *= a;
    y *= a;
    return *this;
  }
  __host__ __device__ float Length() const { return hypot(x, y); }
  __host__ __device__ float SquareLength() const { return x * x + y * y; }
  __host__ __device__ Point Blend(const Point& a, float k) const {
    return {x * (1 - k) + a.x * k, y * (1 - k) + a.y * k};
  }
  __host__ __device__ void Move(const Point& a, float k) {
    x += a.x * k;
    y += a.y * k;
  }
  __host__ __device__ float Dot(const Point& a) const {
    return x * a.x + y * a.y;
  }
  __host__ __device__ void Rot_(float a) {
    float s, c, t;
    sincosf(a, &s, &c);
    t = x;
    x = c * x + s * y;
    y = -s * t + c * y;
  }
  __host__ __device__ float Cross(const Point& a) const {
    return x * a.y - y * a.x;
  }
  __host__ __device__ float Dir() const { return atan2(y, x); }
};

void GetPolylineLengths(float* output, const Point* line, uint size);
void GetPolylineDirections(float* output, const Point* line, uint size);
__host__ __device__ int dcmp(float x);
// 用计算几何中的射线法判断点是否在任意多边形内部（不含边）
// 取水平向右的射线，计算与多边形边的交点数，根据结果的奇偶性判断是否在内部
// 请保证positions各点顺序给出，且第一点与最后一点相同
__host__ __device__ bool InPolygon(const Point& a, const Point*, uint n);
#endif

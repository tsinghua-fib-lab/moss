#include <cmath>
#include "utils/geometry.cuh"

void GetPolylineLengths(float* output, const Point* line, uint size) {
  double s = 0;
  const Point *p = line, *q = p + 1;
  output[0] = 0;
  for (uint i = 1; i < size; ++i) {
    s += std::hypot((double)p->x - q->x, (double)p->y - q->y);
    output[i] = (float)s;
    p = q;
    ++q;
  }
}

void GetPolylineDirections(float* output, const Point* line, uint size) {
  const Point *p = line, *q = p + 1;
  --size;
  for (uint i = 0; i < size; ++i) {
    output[i] = (float)std::atan2((double)q->y - p->y, (double)q->x - p->x);
    p = q;
    ++q;
  }
}

__host__ __device__ int dcmp(float x) {
  return x < 1e-6 ? x > -1e-6 ? 0 : -1 : 1;
}

__host__ __device__ bool InPolygon(const Point& a, const Point* p, uint len) {
  bool flag = false;
  int i = 0, j = len - 2;
  while (i < len - 1) {
    auto& p1 = p[i];
    auto& p2 = p[j];
    if (dcmp((p1 - a).Cross(p2 - a)) == 0 && dcmp((p1 - a).Dot(p2 - a)) <= 0) {
      return false;
    }
    if (((dcmp(p1.y - a.y) > 0) != (dcmp(p2.y - a.y) > 0)) &&
        dcmp((a.x - p1.x) - (p1.x - p2.x) * (a.y - p1.y) / (p1.y - p2.y)) < 0) {
      flag = !flag;
    }
    j = i;
    i++;
  }
  return flag;
}

import bisect
from math import atan2, cos, floor
from math import pi as PI
from math import sin, sqrt

import numpy as np
from shapely import LineString

PI2 = PI * 2
SAMPLE_RATE = 0.25


def simplify_coords(xy, tolerance):
    return list(LineString(xy).simplify(tolerance).coords)


def frange(a, b, s):
    """range for float numbers. `[a, b)` with step `s`"""
    for i in range(floor((b - a) / s) + 1):
        yield a + i * s


class Point:
    def __init__(self, x, y, angle=None, norm=None):
        self.x = x
        self.y = y
        self._angle = angle
        self._norm = norm

    @staticmethod
    def from_angle(angle):
        """Create point on unit circle by angle"""
        return Point(cos(angle), sin(angle), (angle + PI) % (2 * PI) - PI, 1)

    @staticmethod
    def from_node(node):
        return Point(node["x"], node["y"])

    def __mul__(self, x):
        """Scalar product"""
        return Point(self.x * x, self.y * x, self._angle)

    def __matmul__(self, o):
        """Dot product"""
        return self.x * o.x + self.y * o.y

    def __truediv__(self, x):
        """Scalar division"""
        return Point(self.x / x, self.y / x, self._angle)

    @property
    def perp(self):
        """Perpendicular point (-y, x)"""
        return Point(-self.y, self.x, None, self._norm)

    def __add__(self, o):
        if isinstance(o, Point):
            return Point(self.x + o.x, self.y + o.y)
        else:
            return Point(self.x + o, self.y + o)

    def __radd__(self, o):
        if isinstance(o, Point):
            return Point(self.x + o.x, self.y + o.y)
        else:
            return Point(self.x + o, self.y + o)

    def __sub__(self, o):
        if isinstance(o, Point):
            return Point(self.x - o.x, self.y - o.y)
        else:
            return Point(self.x - o, self.y - o)

    def __iter__(self):
        return iter(self.xy)

    @property
    def norm(self):
        if self._norm is None:
            self._norm = sqrt(self.x * self.x + self.y * self.y)
        return self._norm

    def normalize(self):
        l = self.norm
        return Point(self.x / l, self.y / l, self._angle, 1)

    def normalize_(self):
        l = self.norm
        self.x /= l
        self.y /= l
        self._norm = 1
        return self

    def neg_(self):
        self.x = -self.x
        self.y = -self.y
        self._angle = None
        return self

    @property
    def angle(self):
        if self._angle is None:
            self._angle = atan2(self.y, self.x)
        return self._angle

    def close_to(self, o, eps=1e-4):
        return (self - o).norm < eps

    def rotate(self, angle):
        c, s = cos(angle), sin(angle)
        return Point(self.x * c - self.y * s, self.x * s + self.y * c)

    def rotate_(self, angle):
        c, s = cos(angle), sin(angle)
        self.x, self.y = self.x * c - self.y * s, self.x * s + self.y * c
        return self

    @property
    def xy(self):
        return [self.x, self.y]

    def __repr__(self):
        return f"({self.x:.4f}, {self.y:.4f})"


def solve_intersection(a: Point, p: Point, b: Point, q: Point, single=True):
    """Solve for `a + t p = b + s q`. Return `t` if `single` else `(t, s)`"""
    a, b, c = p, q, b - a
    det = a.y * b.x - a.x * b.y
    if abs(det) < 1e-4:
        raise ArithmeticError(f"Cannot solve: det={det}")
    if single:
        return (c.y * b.x - c.x * b.y) / det
    else:
        return (c.y * b.x - c.x * b.y) / det, (c.y * a.x - c.x * a.y) / det


def angle_normalize(angle):
    """Return angle in `[-π, π)`"""
    return (angle + PI) % PI2 - PI


def shoelace(arr):
    """
    Use the shoelace formula to calculate polygon area

    `arr` is a list of `Point`s, vertexes of the polygon
    """
    return abs(sum(i.x * j.y - i.y * j.x for i, j in zip(arr, arr[1:] + [arr[0]])))


class Polyline:
    def __init__(self, xy, simplify=False, allow_overlap=False):
        assert all(isinstance(i, Point) for i in xy)
        # assert allow_overlap or all((i - j).norm > 1e-3 for i, j in zip(xy, xy[1:]))  # 掉头lane存在很短的, assert不通过
        self.xy = xy
        self._sIndex = None
        if simplify:
            self.simplify_()

    @staticmethod
    def from_points(xy, simplify=False):
        assert len(xy) > 1
        p = Polyline([Point(x, y) for x, y in xy])
        if simplify:
            p.simplify_()
        return p

    @staticmethod
    def from_nodes(nodes, simplify=True):
        assert len(nodes) > 1
        p = Polyline([Point(i["x"], i["y"]) for i in nodes])
        if simplify:
            p.simplify_()
        return p

    @staticmethod
    def from_pb(nodes, simplify=True):
        assert len(nodes) > 1
        p = Polyline([Point(i.x, i.y) for i in nodes])
        if simplify:
            p.simplify_()
        return p

    @staticmethod
    def from_linestring(ls):
        return Polyline([Point(*i) for i in ls.coords])

    def offset(self, d):
        """Offset polyline to the left by `d`"""
        return Polyline.from_linestring(LineString(self.xy).offset_curve(d))

    def offset_(self, d):
        self.xy = self.offset(d).xy
        self._sIndex = None
        return self

    @property
    def length(self):
        if self._sIndex is None:
            return sum((b - a).norm for a, b in zip(self.xy, self.xy[1:]))
        else:
            return self._sIndex[-1]

    @staticmethod
    def _ltrim(xy, l):
        assert l > 0, l
        s = 0
        for i, (a, b) in enumerate(zip(xy, xy[1:])):
            d = (b - a).norm
            if s + d > l:
                break
            s += d
        else:
            raise ValueError(f"Cannot trim polyline more than its own length: {l}>{s}")
        return [a + (b - a) * ((l - s) / d)] + xy[i + 1:]

    def ltrim_(self, l):
        """Trim polyline from the left by `l`"""
        self.xy = Polyline._ltrim(self.xy, l)
        return self

    def rtrim_(self, l):
        """Trim polyline from the right by `l`"""
        self.xy = Polyline._ltrim(self.xy[::-1], l)[::-1]
        return self

    @staticmethod
    def connect(A: Point, a: Point, B: Point, b: Point, allow_overlap=False):
        """Return a polyline that connects point `A` and `B` with directions `a` and `b` respectively"""
        assert a.norm == 1
        assert b.norm == 1
        while True:
            c = B - A
            if c.norm < 1:
                break
            p = Point.from_angle((a.angle + c.angle) / 2)
            q = Point.from_angle((b.angle + c.angle) / 2)
            det = q.y * p.x - q.x * p.y
            if abs(det) < 0.01:
                break
            s = (q.y * c.x - q.x * c.y) / det
            t = (p.y * c.x - p.x * c.y) / det
            if s < 0:
                p.neg_()
                s = -s
            if t < 0:
                q.neg_()
                t = -t
            if (
                s < 0.1 * min(1, c.norm)
                or s > 2 * c.norm
                or t < 0.1 * min(1, c.norm)
                or t > 2 * c.norm
            ):
                break
            C = A + p * s
            assert C.close_to(B + q * t)
            if a.perp @ p < 0:
                t1 = a.angle
                t2 = c.angle + PI2
                while t2 > t1:
                    t2 -= PI2
                t3 = b.angle + PI2
                while t3 > t2:
                    t3 -= PI2
                curv1 = -2 * abs(sin(p.angle - c.angle)) / s
                curv2 = -2 * abs(sin(q.angle - c.angle)) / t
                l1 = (t2 - t1) / curv1
                l2 = (t3 - t2) / curv2
            else:
                t1 = a.angle
                t2 = c.angle - PI2
                while t2 < t1:
                    t2 += PI2
                t3 = b.angle - PI2
                while t3 < t2:
                    t3 += PI2
                curv1 = 2 * abs(sin(p.angle - c.angle)) / s
                curv2 = 2 * abs(sin(q.angle - c.angle)) / t
                l1 = (t2 - t1) / curv1
                l2 = (t3 - t2) / curv2
            assert C.close_to(
                A
                + Point(sin(l1 * curv1) / curv1, (1 - cos(l1 * curv1)) / curv1).rotate_(
                    a.angle
                )
            )
            assert B.close_to(
                C
                + Point(sin(l2 * curv2) / curv2, (1 - cos(l2 * curv2)) / curv2).rotate_(
                    t2
                )
            )
            if l1 + l2 > 3 * c.norm or abs(t3 - t1) > PI * 1.5:
                break

            poly = [A]
            step = SAMPLE_RATE
            i = 0
            for i in frange(step, l1, step):
                t = i * curv1
                poly.append(
                    A + Point(sin(t) / curv1, (1 - cos(t)) / curv1).rotate_(a.angle)
                )
            for i in frange(i + step - l1, l2 - step / 2, step):
                t = i * curv2
                poly.append(C + Point(sin(t) / curv2, (1 - cos(t)) / curv2).rotate_(t2))
            poly.append(B)
            return Polyline(poly, True)

        return Polyline([A, B], allow_overlap=allow_overlap)

    @staticmethod
    def connect_polyline(l1, l2):
        return Polyline.connect(l1.xy[-1], Point.from_angle(l1.angle_out), l2.xy[0], Point.from_angle(l2.angle_in))

    def overlap(self, other):
        """
        Get overlaps with another polyline.

        Return the distances of the overlaps along both polyline.
        """
        ret = []
        # ps = []
        l1 = 0
        for a, b in zip(self.xy, self.xy[1:]):
            p = b - a
            pn = p.norm
            l2 = 0
            for c, d in zip(other.xy, other.xy[1:]):
                q = d - c
                qn = q.norm
                try:
                    t, s = solve_intersection(a, p, c, q, False)
                except ArithmeticError:
                    continue
                if 0 <= t <= 1 and 0 <= s <= 1:
                    ret.append([l1 + pn * t, l2 + qn * s])
                    # ps.append(a + p * t)
                l2 += qn
            l1 += pn
        return ret  # , ps

    def _build_s_index(self):
        self._sIndex = [0] + np.cumsum(
            [(a - b).norm for a, b in zip(self.xy, self.xy[1:])]
        ).tolist()

    def get_xy_from_s(self, s, clip=False):
        """Get Point(x, y) given distance traveled"""
        if self._sIndex is None:
            self._build_s_index()
        if not 0 <= s <= self._sIndex[-1]:
            if clip and 0 <= s <= self._sIndex[-1] + 0.01:
                return self.xy[-1]
            else:
                raise ValueError(f"s={s} it out of range [0, {self._sIndex[-1]}]")
        i = bisect.bisect_right(self._sIndex, s) - 1
        k = s - self._sIndex[i]
        return self.xy[i] * (1 - k) + self.xy[i + 1] * k

    def simplify_(self):
        self.xy = [
            Point(x, y)
            for x, y in simplify_coords(
                [[j.x, j.y] for j in self.xy], SAMPLE_RATE
            )
        ]
        return self

    def append(self, p: Point, simplify=True):
        assert isinstance(p, Point)
        self.xy.append(p)
        if simplify:
            self.xy = [
                Point(x, y)
                for x, y in simplify_coords(
                    [[j.x, j.y] for j in self.xy], SAMPLE_RATE
                )
            ]
        return self

    def prepend(self, p: Point, simplify=True):
        assert isinstance(p, Point)
        self.xy = [p] + self.xy
        if simplify:
            self.xy = [
                Point(x, y)
                for x, y in simplify_coords(
                    [[j.x, j.y] for j in self.xy], SAMPLE_RATE
                )
            ]
        return self

    def extend_(self, x):
        """
        extend polyline by `abs(x)`

        if x>=0 then extend on the end, otherwise on the beginning
        """
        if x >= 0:
            a, b = self.xy[-2:]
            self.xy[-1] += (b - a).normalize() * x
        else:
            a, b = self.xy[:2]
            self.xy[0] += (b - a).normalize() * x
        return self

    def numpy(self):
        return np.array([[p.x, p.y] for p in self.xy])

    @property
    def nodes(self):
        return [{"x": i.x, "y": i.y} for i in self.xy]

    @property
    def angle_in(self):
        assert len(self.xy) >= 2
        return (self.xy[1] - self.xy[0]).angle

    @property
    def angle_out(self):
        assert len(self.xy) >= 2
        return (self.xy[-1] - self.xy[-2]).angle

    def __getitem__(self, n):
        return self.xy[n]

    def __setitem__(self, n, p):
        self.xy[n] = p

    def __len__(self):
        return len(self.xy)

    def __repr__(self):
        return "[" + ", ".join(repr(i) for i in self.xy) + "]"

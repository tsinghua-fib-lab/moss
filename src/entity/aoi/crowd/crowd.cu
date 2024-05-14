#include <cmath>
#include "entity/aoi/aoi.cuh"
#include "entity/aoi/crowd/crowd.cuh"
#include "entity/person/person.cuh"
#include "rand/rand.cuh"
#include "simulet.cuh"
#include "utils/geometry.cuh"
#include "utils/macro.h"

namespace simulet::crowd {
// 行人在室内的闲逛的状态转移概率(index与enum StrollTarget对应)
__managed__ float CDF_STROLL[] = {0.5, 0.8, 1};

__device__ bool AtAnyGate(const Crowd& crowd, const Point& p) {
  for (auto& g : crowd.gates) {
    if ((g - p).Length() * 2 < GATE_SIZE) {
      return true;
    }
  }
  return false;
}

__host__ __device__ Point GetRandomPosition(const Crowd& crowd,
                                            rand::Rng64& rng) {
  auto& b = crowd.boundary;
  if (b.size == 1) {
    return b[0];
  }
  if (b.size == 2) {
    return b[0].Blend(b[1], rng.Rand());
  }
  bool in_aoi = false;
  Point p, p0 = crowd.boundary[0];
  for (int i = 0; !in_aoi && i < TRY_GET_RANDOM_POINT; ++i) {
    auto index = rng.RandIntCDF(crowd.area_cdf.data, crowd.area_cdf.size);
    auto a = crowd.boundary[index + 1], b = crowd.boundary[index + 2];
    auto x = rng.Rand(), y = rng.Rand();
    if (x + y >= 1) {
      x = 1 - x;
      y = 1 - y;
    }
    p = p0.Blend(a, x * .9 + .05) + p0.Blend(b, y * .9 + .05) - p0;
    in_aoi = InPolygon(p, crowd.boundary.data, crowd.boundary.size);
  }
  if (!in_aoi) {
    printf("Warning: generated a point on the boundary of aoi\n");
    return p0;
  }
  return p;
}

__device__ void Crowd::WalkToSleep(Person* p) {
  persons.Append({
      .id = p->id,
      .runtime =
          {
              .status = crowd::Status::IDLE,
              .position = {p->runtime.x, p->runtime.y},
          },
      .destination = sleep_points[p->rng.RandInt(sleep_points.size)],
      .desired_speed = crowd::DEFAULT_DESIRED_SPEED,
      .interest = crowd::InterestType::SLEEP,
      .person = p,
  });
}

__device__ void Crowd::WalkToGate(Person* p, AoiGate* gate, bool is_veh) {
  persons.Append({
      .id = p->id,
      .runtime =
          {
              .status = crowd::Status::IDLE,
              .position = {p->runtime.x, p->runtime.y},
          },
      .destination = {gate->x, gate->y},
      .desired_speed = crowd::DEFAULT_DESIRED_SPEED,
      .interest = is_veh ? crowd::InterestType::DRIVING_GATE
                         : crowd::InterestType::WALKING_GATE,
      .person = p,
      .gate = gate,
  });
}

// 每个行人的更新
__device__ void CrowdPersonUpdate(const Crowd& crowd, CrowdPerson& p) {
  switch (p.runtime.status) {
    case Status::IDLE: {
      if (p.stroll_info.target == StrollTarget::PERSON) {
        // 跟随别人，更新目标位置
        if (p.stroll_info.person->id != p.stroll_info.person_id) {
          // 跟随的人不在快照里了，进入PAUSE状态
          p.runtime.status = Status::PAUSE;
          return;
        }
        p.destination = p.stroll_info.person->snapshot.position;
      }
      // 计算社会力，更新行人位置
      auto& position = p.runtime.position;
      auto& velocity = p.runtime.velocity;
      auto& destination = p.destination;
      auto& boundary = crowd.boundary;
      // 记录当前行人所受合力
      Point force{};

      // 计算期望移动目标提供的吸引力
      auto desired_destination = destination - position;
      auto distance = max(1e-5, desired_destination.Length());
      auto desired_force =
          desired_destination * (p.desired_speed / distance) - velocity;
      force.Move(desired_force, 2);  // scale=2 表征吸引力的强度

      // 计算其他行人提供的排斥力
      for (auto& o : crowd.persons) {
        auto& id = o.id;
        auto& snapshot = o.snapshot;
        if (snapshot.status == Status::REACH_TARGET || id == p.id) {
          continue;
        }
        auto& pos = snapshot.position;
        auto& vec = snapshot.velocity;
        auto vr = pos - position;
        auto vv = vec - velocity;
        auto r = max(vr.Length(), 1e-5f);
        auto v = max(vv.Length(), 1e-5f);
        auto c = vr.Dot(vv) / r / v;
        // 之前手动调出的结果，若有需要可以将此超参的调整接口暴露出来
        const float A = 7.55, B = -3, C = .2, D = -.3, theta = 56 * M_PI / 180;
        auto f = A * exp(B * r + C * c + D * r * c);
        auto direction = vr * (-1 / r);
        direction = Point{
            sinf(M_PI / 2 - theta) * direction.x - sinf(theta) * direction.y,
            sinf(theta) * direction.x + sinf(M_PI / 2 - theta) * direction.y,
        };
        force.Move(direction, f);
      }

      // 计算AOI边界产生的排斥力
      // 找到障碍物边界上距离最近的点
      // TODO(yuzihan):在狭窄的地方可能发生表现不自然的问题。考虑改成在视野内寻找距离最近的点，但想不出算法怎么写
      float min_dis = -1;
      Point closest_point;
      for (int i = 0, j = boundary.size - 1; i < boundary.size; j = i++) {
        auto &p1 = boundary[i], &p2 = boundary[j];
        auto rn = (position - p1).Dot(p2 - p1);
        auto rd = (p2 - p1).SquareLength();
        float r;
        if (rd < 1e-4) {
          // p1和p2基本重合，p1~p2不算做障碍物边界
          continue;
        } else if (rn < 0) {
          // p与线段p1~p2的最近点为p1
          r = 0;
        } else if (rn > rd) {
          // p与线段p1~p2的最近点为p2
          r = 1;
        } else {
          // p与线段p1~p2的最近点为二者中间某点
          r = rn / rd;
        }
        auto cp = p1.Blend(p2, r);
        auto md = (cp - position).Length();
        if (min_dis < 0 || md < min_dis) {
          min_dis = md;
          closest_point = cp;
        }
      }
      min_dis = max(1e-5, min_dis);
      // 计算该最近点产生的排斥力，若最近障碍点在门附近则不产生排斥力
      if (min_dis > 0 && !AtAnyGate(crowd, closest_point)) {
        // 短程力
        const float ShortA = 60, ShortB = -3, ShortTheta = 45 * M_PI / 180;
        auto f = ShortA * expf(ShortB * min_dis);
        closest_point.Move(position, -1);
        auto direction = closest_point / -min_dis;
        auto v_desired = destination - position;
        // flag = 0: 行人正要远离障碍物，排斥力沿障碍物法向
        // flag = 1: 行人正要靠近障碍物，排斥力偏转，促使行人从左侧绕开障碍物
        // flag = -1: 行人正要靠近障碍物，排斥力偏转，促使行人从右侧绕开障碍物
        int flag = v_desired.Dot(closest_point) > 0;
        if (flag) {
          if (v_desired.Cross(closest_point) > 0) {
            flag = -flag;
          }
          direction.Rot_(flag > 0 ? ShortTheta : -ShortTheta);
        }
        force.Move(direction, f);

        // 长程力
        const float LongA = .1, LongB = -.5, LongTheta = 75 * M_PI / 180;
        f = LongA * expf(LongB * min_dis);
        direction = closest_point / -min_dis;
        if (flag) {
          direction.Rot_(flag > 0 ? LongTheta : -LongTheta);
        }
        force.Move(direction, f);
      }

      auto acc = force;
      velocity.Move(acc, STEP_INTERVAL_INDOOR);
      auto speed = velocity.Length();
      if (speed > 1.2 * p.desired_speed) {
        velocity *= 1.2 * p.desired_speed / speed;
      }
      position.Move(velocity, STEP_INTERVAL_INDOOR);

      // TODO:以小概率转为原地站定
      if (p.person->rng.PTrue(.01)) {
        p.runtime.status = Status::PAUSE;
      }

      // 到达终点
      if ((position - p.destination).Length() < 5) {
        if (p.stroll_info.target == StrollTarget::UNSET) {
          p.runtime.status = Status::REACH_TARGET;
          return;
        }
        switch (p.stroll_info.target) {
          case StrollTarget::NONE:
            p.runtime.status = Status::REACH_TARGET;
            break;
          case StrollTarget::RANDOM_POSITION:
            p.runtime.status = Status::PAUSE;
            break;
          default:;
        }
      }
    } break;
    case Status::PAUSE: {
      p.runtime.velocity = {};
      if (p.stroll_info.target != StrollTarget::UNSET) {
        return;
      }
      // TODO:以小概率转为开始运动
      if (p.person->rng.PTrue(P_PAUSE_TO_IDLE)) {
        p.runtime.status = Status::IDLE;
      }
    } break;
    case Status::REACH_TARGET:;
  }
}

void Init(Simulet* S, const PbAoi& pb, Aoi& a) {
  auto& c = a.crowd;
  c.rng.SetSeed(S->seed++);
  // 初始化容器
  c.persons.mem = S->mem;
  // gates
  {
    auto& g = c.gates;
    g.New(S->mem, a.driving_gates.size + a.walking_gates.size);
    uint i = 0;
    for (auto& p : a.driving_gates) {
      g[i++] = {p.x, p.y};
    }
    for (auto& p : a.walking_gates) {
      g[i++] = {p.x, p.y};
    }
  }
  // boundary
  auto& b = c.boundary;
  {
    b.New(S->mem, pb.positions_size());
    uint index = 0;
    for (auto& p : pb.positions()) {
      b[index++] = {float(p.x()), float(p.y())};
    }
  }
  // 计算以 p0 为顶点对 Aoi 区域进行三角划分后的 n-2 个三角形的面积
  assert(b.size >= 3);
  auto& p0 = b[0];
  // 为了便于概率采样，计算CDF并归一化
  c.area_cdf.New(S->mem, b.size - 2);
  {
    double s = 0;
    for (int i = 1; i < b.size - 1; i++) {
      auto& p1 = b[i];
      auto& p2 = b[i + 1];
      c.area_cdf[i - 1] = s += .5 * abs((p1 - p0).Cross(p2 - p0));
    }
    for (auto& i : c.area_cdf) {
      i /= s;
    }
  }
  // sleep_points
  std::vector<Point> sleep;
  for (int i = 0, j = b.size - 2; i < b.size - 1; j = i, ++i) {
    auto &p1 = b[j], &p2 = b[i], &p3 = b[i + 1];
    auto p12 = max((p1 - p2).Length(), 1e-5),
         p23 = max((p2 - p3).Length(), 1e-5);
    auto s = (p2 - p1).Cross(p3 - p2) / p12 / p23;
    if (s < -.98) {
      auto pp = p2 + (p1 - p2) * (5 / p12) + (p3 - p2) * (5 / p23);
      if (InPolygon(pp, b.data, b.size)) {
        sleep.push_back(pp);
      }
    }
  }
  for (int i = sleep.size(); i < 3; ++i) {
    sleep.push_back(GetRandomPosition(c, c.rng));
  }
  c.sleep_points.New(S->mem, sleep.size());
  {
    uint index = 0;
    for (auto& p : sleep) {
      c.sleep_points[index++] = p;
    }
  }
}

__device__ void Prepare(Aoi& a) {
  auto& c = a.crowd;
  // 更新人的位置
  for (auto& p : c.persons) {
    if (p.stroll_info.target == StrollTarget::UNSET ||
        p.runtime.status != Status::PAUSE) {
      continue;
    }
    // 维护到达中间节点的闲逛行人的状态
    auto& rng = p.person->rng;
    p.stroll_info.target = StrollTarget(1 + rng.RandIntCDF(CDF_STROLL, 3));
    switch (p.stroll_info.target) {
      case StrollTarget::UNSET:
        assert(false);
      case StrollTarget::NONE:
        p.destination = rng.Choose(c.sleep_points);
        p.runtime.status = Status::IDLE;
        break;
      case StrollTarget::RANDOM_POSITION:
        p.destination = GetRandomPosition(c, rng);
        p.runtime.status = Status::IDLE;
        break;
      case StrollTarget::PERSON: {
        if (c.persons.size == 1) break;
        auto index = rng.RandInt(c.persons.size);
        if (c.persons.data + index == &p) {
          index = (index + 1) % c.persons.size;
        }
        auto& fp = c.persons[index];
        p.stroll_info.person = &fp;
        p.destination = fp.runtime.position;
        p.runtime.status = Status::IDLE;
      } break;
    }
  }
}

__device__ void Update(Aoi& a, float step_interval) {
  auto& c = a.crowd;
  for (int step = max(1, int(step_interval / STEP_INTERVAL_INDOOR)); step;
       --step) {
    for (auto& p : c.persons) {
      p.snapshot = p.runtime;
    }
    for (auto& p : c.persons) {
      CrowdPersonUpdate(c, p);
    }
  }
  // 检查人是否到终点并进行人的流转
  for (int i = 0; i < c.persons.size;) {
    auto& cp = c.persons[i];
    auto& p = *cp.person;
    p.runtime.x = cp.runtime.position.x;
    p.runtime.y = cp.runtime.position.y;
    p.runtime.speed = cp.runtime.velocity.Length();
    if (p.runtime.speed > 1e-5) {
      p.runtime.dir = cp.runtime.velocity.Dir();
    }
    if (cp.runtime.status == Status::REACH_TARGET) {
      switch (cp.interest) {
        case InterestType::WALKING_GATE: {
          p.runtime.status = PersonStatus::WAITING_FOR_LEAVING;
          p.runtime.aoi->ped_for_leaving.Append(&p);
        } break;
        case InterestType::DRIVING_GATE: {
          p.runtime.status = PersonStatus::WAITING_FOR_LEAVING;
          p.runtime.aoi->veh_for_leaving.Append(&p);
        } break;
        case InterestType::SLEEP: {
          p.runtime.status = PersonStatus::SLEEP;
          HeapPush(a.sleeping_person, {p.GetDepartureTime(), &p});
        } break;
      }
      c.persons.Delete(i);
    } else {
      ++i;
    }
  }
}
};  // namespace simulet::crowd

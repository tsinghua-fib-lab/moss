#include <unordered_map>
#include <vector>
#include "entity/lane/lane.cuh"
#include "entity/person/person.cuh"
#include "entity/road/road.cuh"
#include "output.cuh"
#include "protos.h"
#include "simulet.cuh"
#include "utils/debug.cuh"
#include "utils/geometry.cuh"
#include "utils/utils.cuh"

namespace simulet {

__device__ bool Lane::IsNoEntry() {
  return !parent_is_road && light_state != LightState::LIGHT_STATE_GREEN;
}

__device__ __host__ void Lane::GetPosition(float s, float& x, float& y) {
  uint i = Clamp<uint>(Search(line_lengths.data, line_lengths.size, s), 1,
                       line_lengths.size - 1);
  auto &p1 = line[i - 1], p2 = line[i];
  float k = (s - line_lengths[i - 1]) / (line_lengths[i] - line_lengths[i - 1]);
  x = p1.x * (1 - k) + p2.x * k;
  y = p1.y * (1 - k) + p2.y * k;
}

__device__ void Lane::GetPositionDir(float s, float& x, float& y, float& dir) {
  uint i = Clamp<uint>(Search(line_lengths.data, line_lengths.size, s), 1,
                       line_lengths.size - 1);
  auto &p1 = line[i - 1], p2 = line[i];
  float k = (s - line_lengths[i - 1]) / (line_lengths[i] - line_lengths[i - 1]);
  x = p1.x * (1 - k) + p2.x * k;
  y = p1.y * (1 - k) + p2.y * k;
  dir = line_directions[i - 1];
}

std::tuple<double, double, double> Lane::GetPositionDir(float s) {
  uint i = Clamp<uint>(Search(line_lengths.data, line_lengths.size, s), 1,
                       line_lengths.size - 1);
  auto &p1 = line[i - 1], p2 = line[i];
  float k = (s - line_lengths[i - 1]) / (line_lengths[i] - line_lengths[i - 1]);
  return {p1.x * (1 - k) + p2.x * k, p1.y * (1 - k) + p2.y * k,
          line_directions[i - 1]};
}

namespace lane {
// 根据(node.s,node.index)排序
__device__ void SortPerson(PersonNode** start, PersonNode** end) {
  for (auto** p = start; p + 1 < end; ++p) {
    auto** m = p;
    auto ms = (**m).s;
    auto mid = (**m).index;
    for (auto** q = p + 1; q < end; ++q) {
      auto qs = (**q).s;
      auto qid = (**q).index;
      if (qs < ms || qs == ms && qid < mid) {
        m = q;
        ms = qs;
        mid = qid;
      }
    }
    if (p != m) Swap(*p, *m);
  }
}

// 合并两个有序链表
__device__ void MergePersonList(PersonNode*& head, PersonNode* other) {
  if (!head) {
    head = other;
    return;
  }
  if (!other) {
    return;
  }
  PersonNode *p, *q, *t;
  if (head->s < other->s) {
    p = head;
    q = other;
  } else {
    p = other;
    q = head;
    head = p;
  }
  while (p->next) {
    if (p->next->s < q->s) {
      p = p->next;
    } else {
      t = p->next;
      p->next = q;
      q->prev = p;
      p = q;
      q = t;
    }
  }
  p->next = q;
  q->prev = p;
}

// 链表删除
__global__ void Prepare0(Lane* lanes, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = lanes[id];
  // 增量更新
  l.ped_cnt += (l.ped_add_buffer.size - l.ped_remove_buffer.size);
  l.veh_cnt += (l.veh_add_buffer.size - l.veh_remove_buffer.size);
  // 删除remove_buffer
  for (auto* p : l.ped_remove_buffer) {
#if !PERF
    assert(p->self->snapshot.lane == &l);
#endif
    ListRemove(p, l.ped_head);
  }
  l.ped_remove_buffer.Clear();
  for (auto* v : l.veh_remove_buffer) {
#if !PERF
    assert(v);
    assert(v->self);
    assert(v->self->snapshot.lane == &l || v->self->snapshot.shadow_lane == &l);
#endif
    ListRemove(v, l.veh_head);
  }
#if !PERF
  assert(!ListCheckLoop(l.veh_head));
#endif
  l.veh_remove_buffer.Clear();
}

// 链表插入和更新
__global__ void Prepare1(Lane* lanes, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = lanes[id];
  // 路口内道路的限行状态与前驱和后继保持一致
  if (!l.parent_is_road && l.type == LaneType::LANE_TYPE_DRIVING) {
    l.restriction = l.successor->restriction;
    l.in_restriction =
        l.predecessor->restriction && l.predecessor->veh_cnt == 0;
  }
#if !PERF
  for (auto* v : l.veh_add_buffer) {
    assert(v->self->runtime.lane == &l || v->self->runtime.shadow_lane == &l);
  }
  for (auto* p : l.ped_add_buffer) {
    assert(p->self->runtime.lane == &l);
  }
#endif
  // 处理链表逆序（更新错误等）
  {
    auto* p = l.ped_head;
    while (p) {
      if (p->next && p->s > p->next->s) {
        auto* q = p->next;
        ListRemove(q, l.ped_head);
        l.ped_add_buffer.AppendNoLock(q);
      } else {
        p = p->next;
      }
    }
    p = l.veh_head;
    while (p) {
      auto* q = p->next;
      if (!q) {
        break;
      }
      if (p->s > q->s) {
        auto* r = p->prev;
        if (!r || r->s <= q->s) {
          // 删除p更划算
          if (p->overwritable) {
            // 若可覆盖则不删除
            p->s = q->s;
          } else {
            q->prev = r;
            (r ? r->next : l.veh_head) = q;
            l.veh_add_buffer.AppendNoLock(p);
          }
        } else {
          // 删除q更划算
          r = p->next = q->next;
          if (r) {
            r->prev = p;
          }
          l.veh_add_buffer.AppendNoLock(q);
          q = r;
        }
      }
      p = q;
    }
  }
  // 排序、建立链表、合并
  if (l.ped_add_buffer) {
    SortPerson(l.ped_add_buffer.begin(), l.ped_add_buffer.end());
    ListLink(l.ped_add_buffer.begin(), l.ped_add_buffer.end());
    MergePersonList(l.ped_head, l.ped_add_buffer[0]);
    l.ped_add_buffer.Clear();
  }
  if (l.veh_add_buffer) {
    SortPerson(l.veh_add_buffer.begin(), l.veh_add_buffer.end());
    ListLink(l.veh_add_buffer.begin(), l.veh_add_buffer.end());
#if !PERF
    assert(!ListCheckLoop(l.veh_add_buffer[0]));
    if (l.veh_add_buffer) {
      auto* p = l.veh_head;
      while (p) {
        for (auto* q : l.veh_add_buffer) {
          assert(p != q);
        }
        p = p->next;
      }
    }
#endif
    MergePersonList(l.veh_head, l.veh_add_buffer[0]);
#if !PERF
    if (ListCheckLoop(l.veh_head)) {
      printf("%d\n", l.veh_add_buffer.size);
    }
    assert(!ListCheckLoop(l.veh_head));
#endif
    l.veh_add_buffer.Clear();
  }

  // 遍历链表，更新LaneObservation
  auto* v = l.veh_head;
  auto *o = l.observations.begin(), *o_end = l.observations.end();
  auto* op = l.observers.begin();
  if (v && o < o_end) {
    while (true) {
      auto s = v->s;
      while (s >= *op) {
        o->front = v;
        o->back = v->prev;
        ++op;
        ++o;
        if (o == o_end) {
          break;
        }
      }
      if (!v->next || o == o_end) {
        break;
      }
      v = v->next;
    }
    while (o < o_end) {
      o->back = v;
      o->front = nullptr;
      ++o;
    }
  }
}

__global__ void Prepare2(Lane* lanes, uint size, bool enable_output) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = lanes[id];
  // 计算压力
  if (l.parent_is_road) {
    int cnt = 0;
    for (auto& i : l.predecessors) {
      if (!i.lane->restriction) {
        cnt += 1;
      }
    }
    l.pressure_in = float(l.veh_cnt) / max(1, cnt) / l.length;
    cnt = 0;
    for (auto& i : l.successors) {
      if (!i.lane->restriction) {
        cnt += 1;
      }
    }
    l.pressure_out = float(l.veh_cnt) / max(1, cnt) / l.length;
  }
  // 构建支链
  PersonNode *p, *pp, *q, *qq;
  if (!l.need_side_update || !(q = l.veh_head) ||
      !(p = l.side_lanes[LEFT]->veh_head)) {
    return;
  }
  pp = qq = nullptr;
  // 当p->s==q->s时，我们认为p在前面，因为中国是从左侧超车
  while (p || q) {
    while (p && (!q || p->s < q->s)) {
      p->sides[RIGHT][BACK] = qq;
      p->sides[RIGHT][FRONT] = q;
      pp = p;
      p = p->next;
    }
    while (q && (!p || q->s <= p->s)) {
      q->sides[LEFT][BACK] = pp;
      q->sides[LEFT][FRONT] = p;
      qq = q;
      q = q->next;
    }
  }
}

__global__ void UpdateLaneStatus(Lane* lanes, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = lanes[id];
  if (!l.parent_is_road) {
    return;
  }
  double v = 0;
  uint n = 0;
  auto* p = l.veh_head;
  while (p) {
    if (!p->is_shadow) {
      v += p->self->snapshot.speed;
      ++n;
    }
    p = p->next;
  }
  atomicAdd(&l.parent_road->v_speed_10000, int(v * 10000));
  atomicAdd(&l.parent_road->v_cnt, n);
}

__global__ void UpdateRoadStatus(Road* roads, uint size, float k,
                                 MArrZ<char>* road_output) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& r = roads[id];
  r.status = k * r.status +
             (1 - k) * (r.v_cnt ? 5 - Clamp<float>(r.v_speed_10000 / 2000.f /
                                                       r.v_cnt / r.max_speed,
                                                   0, 4)
                                : 1);
  (*road_output)[id] = round(r.status);
  r.v_cnt = 0;
  r.v_speed_10000 = 0;
}

__global__ void UpdateTrafficLight(Lane** lanes, uint size,
                                   DVector<output::AgentOutput>* agent_output) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = *lanes[id];
  if (l.need_output) {
    auto& o = agent_output->GetAppend();
    o.type = output::AgentOutputType::TRAFFIC_LIGHT;
    o.l_id = l.id;
    o.light_state = l.light_state;
    o.light_time = l.light_time;
    o.cx = l.center_x;
    o.cy = l.center_y;
  }
}

void Data::Init(Simulet* S, const PbMap& map) {
  this->S = S;
  stream = NewStream();
  lanes.New(S->mem, map.lanes_size());
  SetGridBlockSize(Prepare0, lanes.size, S->sm_count, g_prepare0, b_prepare0);
  SetGridBlockSize(Prepare1, lanes.size, S->sm_count, g_prepare1, b_prepare1);
  SetGridBlockSize(Prepare2, lanes.size, S->sm_count, g_prepare2, b_prepare2);
  SetGridBlockSize(UpdateTrafficLight, output_lanes.size, S->sm_count,
                   g_update_tl, b_update_tl);
  SetGridBlockSize(UpdateLaneStatus, lanes.size, S->sm_count, g_update_ls,
                   b_update_ls);
  SetGridBlockSize(UpdateRoadStatus, S->road.roads.size, S->sm_count,
                   g_update_rs, b_update_rs);
  // 建立映射
  {
    auto* p = lanes.data;
    for (auto& lane : map.lanes()) {
      lane_map[lane.id()] = p++;
    }
  }
  uint index = 0;
  for (auto& pb : map.lanes()) {
    auto& l = lanes[index];
    // 基本属性
    l.id = pb.id();
    l.index = index++;
    l.parent_id = pb.parent_id();
    l.next_road_id = unsigned(-1);
    l.type = pb.type();
    l.turn = pb.turn();
    l.max_speed = pb.max_speed();
    l.width = pb.max_speed();
    // 初始化容器
    l.ped_add_buffer.mem = S->mem;
    l.veh_add_buffer.mem = S->mem;
    l.ped_remove_buffer.mem = S->mem;
    l.veh_remove_buffer.mem = S->mem;
    // 红绿灯
    l.light_state = LightState::LIGHT_STATE_GREEN;
    l.light_time = 1e999;
    // 几何
    auto size = pb.center_line().nodes_size();
    l.line.New(S->mem, size);
    l.line_lengths.New(S->mem, size);
    l.line_directions.New(S->mem, size - 1);
    {
      uint index = 0;
      for (auto& i : pb.center_line().nodes()) {
        auto& p = l.line[index++];
        p.x = i.x();
        p.y = i.y();
      }
    }
    GetPolylineLengths(l.line_lengths.data, l.line.data, l.line.size);
    GetPolylineDirections(l.line_directions.data, l.line.data, l.line.size);
    l.length = l.line_lengths.back();
    l.GetPosition(l.length / 2, l.center_x, l.center_y);
    // 连接关系
    l.predecessors.New(S->mem, pb.predecessors_size());
    {
      uint index = 0;
      for (auto& i : pb.predecessors()) {
        auto& p = l.predecessors[index++];
        p.lane = lane_map.at(i.id());
        p.type = i.type();
      }
    }
    if (l.predecessors) {
      l.predecessor = l.predecessors[0].lane;
    }
    l.successors.New(S->mem, pb.successors_size());
    {
      uint index = 0;
      for (auto& i : pb.successors()) {
        auto& p = l.successors[index++];
        p.lane = lane_map.at(i.id());
        p.type = i.type();
      }
    }
    if (l.successors) {
      l.successor = l.successors[0].lane;
    }
    // overlap
    l.overlaps.New(S->mem, pb.overlaps_size());
    {
      uint index = 0;
      for (auto& o : pb.overlaps()) {
        auto& p = l.overlaps[index++];
        p.other = lane_map.at(o.self().lane_id());
        p.other_s = o.other().s();
        p.self_first = o.self_first();
      }
    }
    // 初始化为0
    l.veh_cnt = 0;
    l.ped_cnt = 0;
    // parent在junction处初始化填充
    l.parent_junction = nullptr;
    // side_lanes待Road初始化时填充
    // TODO: 临时解决方案，预分配内存大小
    l.veh_add_buffer.Reserve(max(S->config.lane_veh_add_buffer_size, l.length));
    l.veh_add_buffer._error_code =
        ErrorCode(ErrorType::LANE_VEH_ADD_BUFFER_FULL, l.id);
    l.veh_remove_buffer.Reserve(
        max(S->config.lane_veh_remove_buffer_size, l.length));
    l.veh_remove_buffer._error_code =
        ErrorCode(ErrorType::LANE_VEH_REMOVE_BUFFER_FULL, l.id);
    if (!S->is_python_api) {
      l.ped_add_buffer.Reserve(500);
      l.ped_remove_buffer.Reserve(500);
    }
  }
}

void Data::PrepareAsync() {
  // 三次prepare需要串行完成
  if (!lanes.size) {
    return;
  }
  Prepare0<<<g_prepare0, b_prepare0, 0, stream>>>(lanes.data, lanes.size);
  Prepare1<<<g_prepare1, b_prepare1, 0, stream>>>(lanes.data, lanes.size);
  Prepare2<<<g_prepare2, b_prepare2, 0, stream>>>(
      lanes.data, lanes.size, S->output.option == output::Option::LANE);
}

void Data::UpdateAsync() {
  if (!lanes.size) {
    return;
  }
  if (S->output.option == output::Option::AGENT) {
    UpdateTrafficLight<<<g_update_tl, b_update_tl, 0, stream>>>(
        output_lanes.data, output_lanes.size, &S->output.M->agent_output);
  } else if (S->output.option == output::Option::LANE) {
    UpdateLaneStatus<<<g_update_ls, b_update_ls, 0, stream>>>(lanes.data,
                                                              lanes.size);
    UpdateRoadStatus<<<g_update_rs, b_update_rs, 0, stream>>>(
        S->road.roads.data, S->road.roads.size, S->road.k_status,
        &S->output.M->road_output);
  }
}

void Data::Save(std::vector<LaneCheckpoint>& state) {
  state.resize(lanes.size);
  for (int i = 0; i < lanes.size; ++i) {
    auto& l = lanes[i];
    auto& s = state[i];
    s.restriction = l.restriction;
    s.ped_head = l.ped_head;
    s.veh_head = l.veh_head;
    s.ped_cnt = l.ped_cnt;
    s.veh_cnt = l.veh_cnt;
    s.pressure_in = l.pressure_in;
    s.pressure_out = l.pressure_out;
    l.observations.Save(s.observations);
    l.ped_add_buffer.Save(s.ped_add_buffer);
    l.veh_add_buffer.Save(s.veh_add_buffer);
    l.ped_remove_buffer.Save(s.ped_remove_buffer);
    l.veh_remove_buffer.Save(s.veh_remove_buffer);
  }
}

void Data::Load(const std::vector<LaneCheckpoint>& state) {
  assert(state.size() == lanes.size);
  for (int i = 0; i < lanes.size; ++i) {
    auto& l = lanes[i];
    auto& s = state[i];
    l.restriction = s.restriction;
    l.ped_head = s.ped_head;
    l.veh_head = s.veh_head;
    l.ped_cnt = s.ped_cnt;
    l.veh_cnt = s.veh_cnt;
    l.pressure_in = s.pressure_in;
    l.pressure_out = s.pressure_out;
    l.observations.Load(s.observations);
    l.ped_add_buffer.Load(s.ped_add_buffer);
    l.veh_add_buffer.Load(s.veh_add_buffer);
    l.ped_remove_buffer.Load(s.ped_remove_buffer);
    l.veh_remove_buffer.Load(s.veh_remove_buffer);
  }
}
}  // namespace lane
}  // namespace simulet

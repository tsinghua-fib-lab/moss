#include <unordered_map>
#include <vector>
#include "entity/lane/lane.cuh"
#include "entity/person/person.cuh"
#include "entity/road/road.cuh"
#include "moss.cuh"
#include "protos.h"
#include "utils/debug.cuh"
#include "utils/geometry.cuh"
#include "utils/utils.cuh"

namespace moss {

__device__ bool Lane::IsNoEntry() {
  return !parent_is_road && light_state != LightState::LIGHT_STATE_GREEN;
}

__host__ __device__ void Lane::GetPosition(float s, float& x, float& y) {
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

// 同一道路上两个车道间按照等比例方式进行投影
// project from src_lane to dest_lane in the same road, using an equal ratio
// way.
__device__ float ProjectFromLane(Lane* src_lane, Lane* dest_lane, float s) {
  assert(src_lane && dest_lane);
  assert(src_lane->parent_is_road && dest_lane->parent_is_road);
  assert(src_lane->parent_id == dest_lane->parent_id);
  return Clamp(s / src_lane->length * dest_lane->length, 0.f,
               dest_lane->length);
}

namespace lane {

// sort by s and index, the index is used to make it deterministic
__device__ bool less(const PersonNode* a, const PersonNode* b) {
  return a->s < b->s || a->s == b->s && a->index < b->index;
}

// use the list to create a sorted linked list of PersonNode (rather than DNode)
// and return the head of the new linked list
// swap sort by (p->data->s, p->data->index)
__device__ moss::PersonNode* InitPersonLink(DList<moss::PersonNode>& list) {
  if (list.head->next == nullptr) {
    return nullptr;
  }
  // sort by s and index
  while (true) {
    bool sorted = true;
    for (auto* prev = list.head; prev; prev = prev->next) {
      auto* p = prev->next;
      if (p->next == nullptr) {
        break;
      }
      auto* q = p->next;
      // swap
      if (less(q->data, p->data)) {
        // Swap the nodes by adjusting the pointers
        prev->next = q;
        p->next = q->next;
        q->next = p;
        sorted = false;
      }
    }
    if (sorted) {
      break;
    }
  }
  // clear next / prev pointers
  for (auto* p = list.head->next; p; p = p->next) {
    p->data->next = nullptr;
    p->data->prev = nullptr;
  }
  // build linked list
  auto* head = list.head->next->data;
  auto* p = head;
  for (auto* q = list.head->next->next; q; q = q->next) {
    p->next = q->data;
    q->data->prev = p;
    p = q->data;
  }
  return head;
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
  if (less(head, other)) {
    p = head;
    q = other;
  } else {
    p = other;
    q = head;
    head = p;
  }
  while (p->next) {
    if (less(p->next, q)) {
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

// Linked-list remove
__global__ void Prepare0(Lane* lanes, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = lanes[id];
  // remove node saved in remove buffer
  for (auto* p = l.ped_remove_buffer.head->next; p; p = p->next) {
    ListRemove(p->data, l.ped_head);
    --l.ped_cnt;
  }
  l.ped_remove_buffer.Clear();
  for (auto* p = l.veh_remove_buffer.head->next; p; p = p->next) {
    ListRemove(p->data, l.veh_head);
    --l.veh_cnt;
  }
#if !PERF
  assert(!ListCheckLoop(l.veh_head));
#endif
  l.veh_remove_buffer.Clear();
  // calculate the added number
  for (auto* p = l.ped_add_buffer.head->next; p; p = p->next) {
    ++l.ped_cnt;
  }
  for (auto* p = l.veh_add_buffer.head->next; p; p = p->next) {
    ++l.veh_cnt;
  }
}

// Linked-list sort, add and merge
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
  for (auto* p = l.veh_add_buffer.head->next; p; p = p->next) {
    auto v = p->data;
    if (p->data->is_shadow) {
      if (v->self->runtime.shadow_lane != &l) {
        printf("Lane: add buffer bad shadow node %d %d %d %d\n",
               v->self->runtime.shadow_lane->id, l.id,
               v->self->runtime.shadow_lane->index, l.index);
      }
      assert(v->self->runtime.shadow_lane == &l);
    } else {
      if (v->self->runtime.lane != &l) {
        printf("Lane: add buffer bad node %d %d %d %d\n",
               v->self->runtime.lane->id, l.id, v->self->runtime.lane->index,
               l.index);
      }
      assert(v->self->runtime.lane == &l);
    }
  }
  for (auto* p = l.ped_add_buffer.head->next; p; p = p->next) {
    assert(p->data->self->runtime.lane == &l);
  }
#endif
  // fix disorder in linked list
  {
    auto* p = l.ped_head;
    while (p) {
      if (p->next && less(p->next, p)) {
        auto* q = p->next;
        ListRemove(q, l.ped_head);
        l.ped_add_buffer.Add(&q->add_node);
      } else {
        p = p->next;
      }
    }
    p = l.veh_head;
    if (p) {
      auto* q = p->next;
      while (q) {
        if (less(q, p)) {
          auto* r = p->prev;
          if (!r || less(r, q)) {
            // 删除p更划算
            // r->p->q
            if (p->overwritable && p->index < q->index) {
              // 若可覆盖则不删除
              p->s = q->s;
            } else {
              q->prev = r;
              (r ? r->next : l.veh_head) = q;
              l.veh_add_buffer.Add(&p->add_node);
            }
            p = q;
            q = p->next;
          } else {
            // 删除q更划算
            // p->q->r
            r = p->next = q->next;
            if (r) {
              r->prev = p;
            }
            l.veh_add_buffer.Add(&q->add_node);
            q = r;
          }
        } else {
          p = q;
          q = p->next;
        }
      }
    }
  }
  // sort buffer, build linked list added, merge
  if (l.ped_add_buffer) {
    auto* add_link = InitPersonLink(l.ped_add_buffer);
    MergePersonList(l.ped_head, add_link);
    l.ped_add_buffer.Clear();
  }
  if (l.veh_add_buffer) {
    auto* add_link = InitPersonLink(l.veh_add_buffer);
#if !PERF
    assert(!ListCheckLoop(add_link));
    if (add_link) {
      auto* p = l.veh_head;
      while (p) {
        for (auto* q = add_link; q; q = q->next) {
          if (p == q) {
            p->PrintDebugString();
            q->PrintDebugString();
            printf("===== Lane Veh Linked List =====\n");
            for (auto* r = l.veh_head; r; r = r->next) {
              r->PrintDebugString();
            }
            printf("===== Add Link =====\n");
            for (auto* r = add_link; r; r = r->next) {
              r->PrintDebugString();
            }
          }
          assert(p != q);
        }
        p = p->next;
      }
    }
#endif
    MergePersonList(l.veh_head, add_link);
#if !PERF
    // if (ListCheckLoop(l.veh_head)) {
    //   printf("%d\n", l.veh_add_buffer.size);
    // }
    assert(!ListCheckLoop(l.veh_head));
#endif
    l.veh_add_buffer.Clear();
  }
}

// build side links and compute pressure
__global__ void Prepare2(Lane* lanes, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = lanes[id];
  // compute pressure for max-pressure control
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
  // build side links
  PersonNode *p, *pp, *q, *qq;
  // the leftest lane does not need to update side links
  if (l.offset_on_road == 0) {
    return;
  }
  q = l.veh_head;
  // the lane is not a driving lane or there is no vehicle
  if (!q) {
    return;
  }
  p = l.side_lanes[LEFT]->veh_head;
  // the left side lane is not a driving lane or there is no vehicle
  if (!p) {
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

__global__ void PrepareOutput(Lane** lanes, uint size, TlOutput* outputs,
                              float t) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& l = *lanes[id];
  auto& o = outputs[id];
  o.t = t;
  o.id = l.id;
  o.state = l.light_state;
}

// calculate average speed of lanes
__global__ void UpdateLaneStatistics(Lane* lanes, uint size, float k) {
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
      v += p->self->snapshot.v;
      ++n;
    }
    p = p->next;
  }
  l.v_avg = k * l.v_avg + (1 - k) * (n ? v / n : l.max_speed);
}

// calculate average speed of roads
__global__ void UpdateRoadStatistics(Road* roads, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& r = roads[id];
  r.v_avg = 0;
  if (!r.lanes.size) {
    return;
  }
  for (auto* l : r.lanes) {
    r.v_avg += l->v_avg;
  }
  r.v_avg /= r.lanes.size;
  r.status = round(5 - Clamp<float>(r.v_avg / r.max_speed * 5, 0, 4));
}

void Data::Init(Moss* S, const PbMap& map) {
  this->S = S;
  stream = NewStream();
  lanes.New(S->mem, map.lanes_size());
  SetGridBlockSize(Prepare0, lanes.size, S->sm_count, g_prepare0, b_prepare0);
  SetGridBlockSize(Prepare1, lanes.size, S->sm_count, g_prepare1, b_prepare1);
  SetGridBlockSize(Prepare2, lanes.size, S->sm_count, g_prepare2, b_prepare2);
  // create lane map
  {
    auto* p = lanes.data;
    for (auto& lane : map.lanes()) {
      lane_map[lane.id()] = p++;
    }
  }
  uint index = 0;
  for (auto& pb : map.lanes()) {
    auto& l = lanes[index];
    // copy basic attributes
    l.id = pb.id();
    l.index = index++;
    l.parent_id = pb.parent_id();
    l.type = pb.type();
    l.turn = pb.turn();
    l.max_speed = l.v_avg = pb.max_speed();
    l.width = pb.width();
    // traffic light
    l.light_state = LightState::LIGHT_STATE_GREEN;
    l.light_time = 1e999;
    // create polyline
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
    // init count
    l.veh_cnt = 0;
    l.ped_cnt = 0;
    // the parent pointer is set in junction initialization
    l.parent_junction = nullptr;
    // side_lanes are set in road initialization

    // init add / remove buffer
    l.ped_add_buffer.Init();
    l.ped_remove_buffer.Init();
    l.veh_add_buffer.Init();
    l.veh_remove_buffer.Init();
  }
  // copy connection (predecessors and successors)
  for (auto& pb : map.lanes()) {
    auto& l = *At(pb.id());
    l.predecessors.New(S->mem, pb.predecessors_size());
    {
      uint index = 0;
      for (auto& i : pb.predecessors()) {
        auto& p = l.predecessors[index++];
        p.lane = At(i.id());
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
        p.lane = At(i.id());
        p.type = i.type();
      }
    }
    if (l.successors) {
      l.successor = l.successors[0].lane;
    }
  }
}

void Data::InitSizes(Moss* S) {
  outputs.New(S->mem, output_lanes.size);
  SetGridBlockSize(PrepareOutput, output_lanes.size, S->sm_count,
                   g_prepare_output, b_prepare_output);
  SetGridBlockSize(UpdateLaneStatistics, lanes.size, S->sm_count, g_update_ls,
                   b_update_ls);
  SetGridBlockSize(UpdateRoadStatistics, S->road.roads.size, S->sm_count,
                   g_update_rs, b_update_rs);
}

void Data::PrepareAsync(cudaStream_t stream) {
  // Sequentially execute the following three kernels
  if (!lanes.size) {
    return;
  }
  Prepare0<<<g_prepare0, b_prepare0, 0, stream>>>(lanes.data, lanes.size);
  Prepare1<<<g_prepare1, b_prepare1, 0, stream>>>(lanes.data, lanes.size);
  Prepare2<<<g_prepare2, b_prepare2, 0, stream>>>(lanes.data, lanes.size);
}

void Data::PrepareOutputAsync(cudaStream_t stream) {
  if (!output_lanes.size) {
    return;
  }
  PrepareOutput<<<g_prepare_output, b_prepare_output, 0, stream>>>(
      output_lanes.data, output_lanes.size, outputs.data, S->time);
}

void Data::UpdateAsync() {
  UpdateLaneStatistics<<<g_update_ls, b_update_ls, 0, stream>>>(
      lanes.data, lanes.size, S->road.k_status);
  UpdateRoadStatistics<<<g_update_rs, b_update_rs, 0, stream>>>(
      S->road.roads.data, S->road.roads.size);
}

}  // namespace lane
}  // namespace moss

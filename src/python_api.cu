#include <cuda.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include "simulet.cuh"
#include "utils/macro.h"

namespace py = pybind11;
using namespace py::literals;
using namespace simulet;
template <class T>
using vec = std::vector<T>;
using std::pair;
using no_gil = py::call_guard<py::gil_scoped_release>;

template <typename T>
inline py::array_t<typename T::value_type> asarray(T* ptr) {
  py::gil_scoped_acquire _;
  auto capsule =
      py::capsule(ptr, [](void* p) { delete reinterpret_cast<T*>(p); });
  return py::array(ptr->size(), ptr->data(), capsule);
}
template <class T>
vec<T>& remove_duplicate(vec<T>& arr) {
  std::sort(arr.begin(), arr.end());
  arr.erase(std::unique(arr.begin(), arr.end()), arr.end());
  return arr;
}

class Engine {
 private:
  Simulet S;

 public:
  Engine(const std::string& map_file, const std::string& agent_file,
         uint start_step, float step_interval, int seed, int verbose_level,
         int agent_limit, bool disable_aoi_out_control, bool disable_junction,
         int junction_blocking_count, float junction_yellow_time,
         float phase_pressure_coeff, uint lane_change_algorithm,
         float mobil_lc_forbidden_distance, uint lane_veh_add_buffer_size,
         uint lane_veh_remove_buffer_size, float speed_stat_interval,
         uint device)
      : S() {
    S.is_python_api = true;
    S.Init({.map_file = map_file,
            .agent_file = agent_file,
            .agent_limit = uint(agent_limit),
            .output_file = "",
            .output_type = "disable",
            .speed_stat_interval = speed_stat_interval,
            .start_step = start_step,
            .total_step = 1 << 30,
            .step_interval = step_interval,
            .routing_url = "",
            .pre_routing = false,
            .enable_aoi_indoor = false,
            .enable_junction = !disable_junction,
            .seed = seed,
            .x_min = 0,
            .y_min = 0,
            .x_max = 0,
            .y_max = 0,
            .verbose_level = verbose_level,
            .disable_aoi_out_control = disable_aoi_out_control,
            .n_workers = 0,
            .junction_blocking_count = uint(junction_blocking_count),
            .junction_yellow_time = junction_yellow_time,
            .phase_pressure_coeff = phase_pressure_coeff,
            .lane_change_algorithm = lane_change_algorithm,
            .mobil_lc_forbidden_distance = mobil_lc_forbidden_distance,
            .lane_veh_add_buffer_size = lane_veh_add_buffer_size,
            .lane_veh_remove_buffer_size = lane_veh_remove_buffer_size,
            .device = device});
  }
  // 获取当前时间
  float get_current_time() { return S.time; }
  // 获取车辆总数
  uint get_vehicle_count() { return S.person.persons.size; }
  // 按顺序获取车辆id列表
  auto get_vehicle_ids() {
    auto* out = new vec<int>();
    out->reserve(S.person.persons.size);
    for (auto& p : S.person.persons) {
      out->push_back(p.id);
    }
    return asarray(out);
  }
  // 获取运行中的车辆的位置信息，(x,y,dir)
  auto get_vehicle_positions() {
    auto* out = new vec<double>();
    out->reserve(S.person.M->veh_cnt * 3);
    for (auto& p : S.person.persons) {
      if (p.runtime.status == PersonStatus::DRIVING) {
        // out->push_back(p.runtime.lane->GetPositionDir(p.runtime.s));
        out->push_back(p.runtime.x);
        out->push_back(p.runtime.y);
        out->push_back(p.runtime.dir);
      }
    }
    return asarray(out);
  }
  // 获取所有车辆的位置信息，(x,y,dir)
  auto get_vehicle_positions_all() {
    auto* out = new vec<double>();
    out->reserve(S.person.persons.size * 3);
    for (auto& p : S.person.persons) {
      out->push_back(p.runtime.x);
      out->push_back(p.runtime.y);
      out->push_back(p.runtime.dir);
    }
    return asarray(out);
  }
  // 获取运行中的车辆的id和位置信息，(id,x,y,dir)
  auto get_vehicle_id_positions() {
    auto* out = new vec<double>();
    out->reserve(S.person.M->veh_cnt * 4);
    for (auto& p : S.person.persons) {
      if (p.runtime.status == PersonStatus::DRIVING) {
        out->push_back(p.id);
        out->push_back(p.runtime.x);
        out->push_back(p.runtime.y);
        out->push_back(p.runtime.dir);
      }
    }
    return asarray(out);
  }
  // 获取车辆运行状态，0、1、2表示未运行、运行、结束
  auto get_vehicle_statuses() {
    auto* out = new vec<int8_t>();
    out->reserve(S.person.M->veh_cnt);
    for (auto& p : S.person.persons) {
      out->push_back(p.runtime.status == PersonStatus::DRIVING    ? 1
                     : p.runtime.status == PersonStatus::FINISHED ? 2
                                                                  : 0);
    }
    return asarray(out);
  }
  auto get_vehicle_raw_statuses() {
    auto* out = new vec<int8_t>();
    out->reserve(S.person.M->veh_cnt);
    for (auto& p : S.person.persons) {
      out->push_back((int8_t)p.runtime.status);
    }
    return asarray(out);
  }
  // 获取车辆加速度、速度、前车距
  auto get_vehicle_avds() {
    vec<float> out;
    out.reserve(get_running_vehicle_count() * 4);
    for (auto& p : S.person.persons) {
      if (p.runtime.status == PersonStatus::DRIVING) {
        out.emplace_back(p.id);
        out.emplace_back(p.runtime.acc);
        out.emplace_back(p.runtime.speed);
        out.emplace_back(p.ahead_dist);
      }
    }
    return out;
  }
  // 获取正在运行的车辆数
  uint get_running_vehicle_count() { return S.person.M->veh_cnt; }
  // 获取结束的车辆数
  uint get_finished_vehicle_count() { return S.person.M->finished_cnt; }
  // 获取结束的车辆的平均旅行时间
  double get_finished_vehicle_average_traveling_time() {
    return S.person.M->finished_cnt
               ? (double)S.person.M->finished_traveling_time /
                     S.person.M->finished_cnt
               : NAN;
  }
  // 获取车辆旅行或出发时间
  auto get_vehicle_traveling_or_departure_times() {
    auto* out = new vec<float>();
    out->reserve(S.person.M->veh_cnt);
    for (auto& p : S.person.persons) {
      out->push_back(p.traveling_or_departure_time);
    }
    return asarray(out);
  }
  // 获取正在跑的车辆平均旅行时间
  double get_running_vehicle_average_traveling_time() {
    return S.person.M->veh_cnt ? S.time + (double)S.person.M->traveling_time /
                                              S.person.M->veh_cnt
                               : NAN;
  }
  // 获取所有已出发车辆平均旅行时间
  double get_departed_vehicle_average_traveling_time() {
    // running+finished
    auto b = S.person.M->veh_cnt + S.person.M->finished_cnt;
    if (b) {
      auto a = (double)S.time * S.person.M->veh_cnt +
               S.person.M->traveling_time + S.person.M->finished_traveling_time;
      return a / b;
    }
    return NAN;
  }
  auto get_vehicle_lanes() {
    return asarray(
        new vec<int>(S.person.veh_lane.begin(), S.person.veh_lane.end()));
  }
  auto get_vehicle_speeds() {
    return asarray(
        new vec<float>(S.person.veh_speed.begin(), S.person.veh_speed.end()));
  }
  auto get_vehicle_distances() {
    return asarray(new vec<float>(S.person.veh_distance.begin(),
                                  S.person.veh_distance.end()));
  }
  auto get_vehicle_total_distances() {
    return asarray(new vec<float>(S.person.veh_total_distance.begin(),
                                  S.person.veh_total_distance.end()));
  }
  uint get_lane_count() { return S.lane.lanes.size; }
  // 按顺序获取车道id列表
  auto get_lane_ids() {
    auto* out = new vec<int>();
    out->reserve(S.lane.lanes.size);
    for (auto& l : S.lane.lanes) {
      out->push_back(l.id);
    }
    return asarray(out);
  }
  // 获取车道上正在等待的车辆数
  auto get_lane_waiting_vehicle_counts(float speed_threshold) {
    auto* out = new vec<int>(S.lane.lanes.size);
    for (int i = 0; i < S.person.veh_lane.size; ++i) {
      auto lane = S.person.veh_lane[i];
      if (lane >= 0 && S.person.veh_speed[i] < speed_threshold) {
        out->at(lane) += 1;
      }
    }
    return asarray(out);
  }
  // 获取道路末端正在等待的车辆数
  auto get_lane_waiting_at_end_vehicle_counts(float speed_threshold,
                                              float distance_to_end) {
    auto* out = new vec<int>(S.lane.lanes.size);
    for (int i = 0; i < S.person.veh_lane.size; ++i) {
      auto lane = S.person.veh_lane[i];
      if (lane >= 0 && S.person.veh_speed[i] < speed_threshold &&
          S.person.veh_distance[i] >=
              S.lane.lanes[i].length - distance_to_end) {
        out->at(lane) += 1;
      }
    }
    return asarray(out);
  }
  // 获取车道上的车辆数
  auto get_lane_vehicle_counts() {
    auto* out = new vec<int>(S.lane.lanes.size);
    for (int i = 0; i < S.person.veh_lane.size; ++i) {
      auto lane = S.person.veh_lane[i];
      if (lane >= 0) {
        out->at(lane) += 1;
      }
    }
    return asarray(out);
  }
  // 获取车道状态，0、1、2、3代表绿、黄、红、限行
  auto get_lane_statuses() {
    auto* out = new vec<int8_t>();
    out->reserve(S.lane.lanes.size);
    for (auto& l : S.lane.lanes) {
      out->push_back((l.restriction | l.in_restriction)                ? 3
                     : l.light_state == LightState::LIGHT_STATE_GREEN  ? 0
                     : l.light_state == LightState::LIGHT_STATE_YELLOW ? 1
                                                                       : 2);
    }
    return asarray(out);
  }
  // 获取车道几何信息，[[x,y,...],...]
  vec<vec<float>> get_lane_geoms() {
    vec<vec<float>> out;
    out.resize(S.lane.lanes.size);
    int i = 0;
    for (auto& l : S.lane.lanes) {
      auto& o = out[i++];
      o.reserve(l.line.size * 2);
      for (auto& p : l.line) {
        o.push_back(p.x);
        o.push_back(p.y);
      }
    }
    return out;
  }
  // 获取车道长度
  auto get_lane_lengths() {
    auto* out = new vec<float>();
    out->reserve(S.lane.lanes.size);
    for (auto& l : S.lane.lanes) {
      out->push_back(l.length);
    }
    return asarray(out);
  }
  // 获取车道平均车速
  auto get_lane_average_vehicle_speed(int lane_index) {
    if (lane_index >= S.lane.lanes.size) {
      throw std::out_of_range("Lane index out of range.");
    }
    return S.lane.lanes[lane_index].v_avg;
  }
  // 获取路口数目
  uint get_junction_count() { return S.junction.junctions.size; }
  // 按顺序获取路口id列表
  auto get_junction_ids() {
    auto* out = new vec<int>();
    out->reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      out->push_back(j.id);
    }
    return asarray(out);
  }
  // 获取路口中的车道index
  auto get_junction_lanes() {
    vec<vec<int>> out;
    out.reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      vec<int> ids;
      ids.reserve(j.lanes.size);
      for (auto* l : j.lanes) {
        if (l->type == LaneType::LANE_TYPE_DRIVING) {
          ids.push_back(l->index);
        }
      }
      out.push_back(std::move(ids));
    }
    return out;
  }
  // 获取路口的进入和离开车道index列表
  auto get_junction_inout_lanes() {
    vec<vec<int>> in, out;
    in.reserve(S.junction.junctions.size);
    out.reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      vec<int> lane_in, lane_out;
      lane_in.reserve(j.lanes.size);
      lane_out.reserve(j.lanes.size);
      for (auto* l : j.lanes) {
        if (l->type == LaneType::LANE_TYPE_DRIVING) {
          lane_in.push_back(l->predecessor->index);
          lane_out.push_back(l->successor->index);
        }
      }
      in.push_back(std::move(remove_duplicate(lane_in)));
      out.push_back(std::move(remove_duplicate(lane_out)));
    }
    return std::make_pair(in, out);
  }
  // 获取路口每个相位对应的进入和离开车道index列表
  auto get_junction_phase_lanes() {
    vec<vec<pair<vec<int>, vec<int>>>> out;
    out.resize(S.junction.junctions.size);
    int i = 0;
    for (auto& j : S.junction.junctions) {
      auto& ps = out[i++];
      for (auto& p : j.tl.phases) {
        vec<int> lane_in, lane_out;
        int k = 0;
        for (auto& pp : p) {
          if (pp == LightState::LIGHT_STATE_GREEN) {
            auto* l = j.lanes[k];
            // TODO: 处理人行道？
            if (l->type == LaneType::LANE_TYPE_DRIVING) {
              lane_in.push_back(l->predecessor->index);
              lane_out.push_back(l->successor->index);
            }
          }
          ++k;
        }
        ps.emplace_back(std::move(remove_duplicate(lane_in)),
                        std::move(remove_duplicate(lane_out)));
      }
    }
    return out;
  }
  // 获取路口的当前相位id
  auto get_junction_phase_ids() {
    auto* out = new vec<int>();
    out->reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      out->push_back(j.tl.ok ? j.tl.phase_index : (unsigned)-1);
    }
    return asarray(out);
  }
  // 获取路口的相位数
  auto get_junction_phase_counts() {
    auto* out = new vec<int>();
    out->reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      out->push_back(j.tl.phases.size);
    }
    return asarray(out);
  }
  // 获取与路口相连的动态道路index
  auto get_junction_dynamic_roads() {
    vec<vec<int>> out(S.junction.junctions.size);
    for (auto& r : S.road.roads) {
      if (r.nrl_ranges) {
        out[r.lanes[0]->successor->parent_junction->index].push_back(r.index);
      }
    }
    return out;
  }
  // 获取路口数目
  uint get_road_count() { return S.road.roads.size; }
  // 获取动态道路的车道方案
  auto get_road_lane_plans(uint road_index) {
    if (road_index >= S.road.roads.size) {
      throw std::out_of_range("road index out of range.");
    }
    auto& r = S.road.roads[road_index];
    vec<vec<pair<int, int>>> out;
    out.resize(r.nrl_ranges.size / 2);
    for (int i = 0; i < r.nrl_ranges.size; i += 2) {
      auto a = r.nrl_ranges[i], b = r.nrl_ranges[i + 1];
      auto& o = out[i / 2];
      for (int j = a; j < b; ++j) {
        o.emplace_back(r.next_road_lanes[j].l1->index,
                       (r.next_road_lanes[j].l2 - 1)->index + 1);
      }
    }
    return out;
  }
  // 获取动态道路的当前车道方案id
  int get_road_lane_plan_index(int road_index) {
    if (road_index >= S.road.roads.size) {
      throw std::out_of_range("Road index out of range.");
    }
    auto& r = S.road.roads[road_index];
    for (int i = 0; i < r.nrl_ranges.size; i += 2) {
      if (r.nrl_ranges[i] == r.nrl_a && r.nrl_ranges[i + 1] == r.nrl_b) {
        return i / 2;
      }
    }
    throw std::out_of_range("Road lane plan out of range.");
  }
  // 获取道路上正在等待的车辆数
  auto get_road_waiting_vehicle_counts(float speed_threshold) {
    auto* out = new vec<int>(S.road.roads.size);
    for (int i = 0; i < S.person.veh_lane.size; ++i) {
      auto lane = S.person.veh_lane[i];
      if (lane >= 0 && S.person.veh_speed[i] < speed_threshold) {
        auto& l = S.lane.lanes[lane];
        if (l.parent_is_road) {
          out->at(l.parent_road->index) += 1;
        }
      }
    }
    return asarray(out);
  }
  // 获取道路上的车辆数
  auto get_road_vehicle_counts() {
    auto* out = new vec<int>(S.road.roads.size);
    for (int i = 0; i < S.person.veh_lane.size; ++i) {
      auto lane = S.person.veh_lane[i];
      if (lane >= 0) {
        auto& l = S.lane.lanes[lane];
        if (l.parent_is_road) {
          out->at(l.parent_road->index) += 1;
        }
      }
    }
    return asarray(out);
  }
  // 获取道路平均车速
  auto get_road_average_vehicle_speed(int road_index) {
    if (road_index >= S.road.roads.size) {
      throw std::out_of_range("Road index out of range.");
    }
    return S.road.roads[road_index].v_avg;
  }
  // 设置人禁用
  void set_vehicle_enable(uint vehicle_index, bool enable) {
    if (vehicle_index >= S.person.persons.size) {
      throw std::out_of_range("Vehicle index out of range.");
    }
    S.person.persons[vehicle_index].enable = enable;
  }
  void set_vehicle_enable_batch(const std::vector<uint>& vehicle_indices,
                                const std::vector<bool>& enable) {
    for (int i = 0, s = vehicle_indices.size(); i < s; ++i) {
      set_vehicle_enable(vehicle_indices[i], enable[i]);
    }
  }
  // 设置信控模式
  void set_tl_policy(uint junction_index, uint policy) {
    if (junction_index >= S.junction.junctions.size) {
      throw std::out_of_range("Junction index out of range.");
    }
    if (policy > 3) {
      throw std::out_of_range("Invalid policy id.");
    }
    S.junction.junctions[junction_index].tl_policy = (TlPolicy)policy;
  }
  void set_tl_policy_batch(const std::vector<uint>& junction_indices,
                           uint policy) {
    for (auto& i : junction_indices) {
      set_tl_policy(i, policy);
    }
  }
  // 设置信控时长
  void set_tl_duration(uint junction_index, int duration) {
    if (junction_index >= S.junction.junctions.size) {
      throw std::out_of_range("Junction index out of range.");
    }
    S.junction.junctions[junction_index].phase_time = duration;
  }
  void set_tl_duration_batch(const std::vector<uint>& junction_indices,
                             uint policy) {
    for (auto& i : junction_indices) {
      set_tl_duration(i, policy);
    }
  }
  // 设置信控相位
  void set_tl_phase(uint junction_index, uint phase_index) {
    if (junction_index >= S.junction.junctions.size) {
      throw std::out_of_range("Junction index out of range.");
    }
    auto& jc = S.junction.junctions[junction_index];
    if (phase_index >= jc.tl.phases.size) {
      throw std::out_of_range("Phase index out of range.");
    }
    jc.tl.next_index = phase_index;
    jc.tl.set_force = true;
  }
  void set_tl_phase_batch(const std::vector<uint>& junction_indices,
                          const std::vector<uint>& phase_indices) {
    for (int i = 0, s = junction_indices.size(); i < s; ++i) {
      set_tl_phase(junction_indices[i], phase_indices[i]);
    }
  }
  // 设置车道禁行
  void set_lane_restriction(uint lane_index, bool flag) {
    if (lane_index >= S.lane.lanes.size) {
      throw std::out_of_range("Lane index out of range.");
    }
    S.lane.lanes[lane_index].restriction = flag;
  }
  void set_lane_restriction_batch(const std::vector<uint>& lane_indices,
                                  const std::vector<bool>& flags) {
    for (int i = 0, s = lane_indices.size(); i < s; ++i) {
      set_lane_restriction(lane_indices[i], flags[i]);
    }
  }
  // 设置车道限速
  void set_lane_max_speed(uint lane_index, float max_speed) {
    if (lane_index >= S.lane.lanes.size) {
      throw std::out_of_range("Lane index out of range.");
    }
    S.lane.lanes[lane_index].max_speed = max_speed;
  }
  void set_lane_max_speed_batch(const std::vector<uint>& lane_indices,
                                const std::vector<float>& max_speeds) {
    for (int i = 0, s = lane_indices.size(); i < s; ++i) {
      set_lane_max_speed(lane_indices[i], max_speeds[i]);
    }
  }
  // 设置动态道路的车道方案
  void set_road_lane_plan(uint road_index, uint plan_index) {
    if (road_index >= S.road.roads.size) {
      throw std::out_of_range("Road index out of range.");
    }
    auto& r = S.road.roads[road_index];
    plan_index *= 2;
    if (plan_index >= r.nrl_ranges.size) {
      throw std::out_of_range("Plan index out of range.");
    }
    r.nrl_a = r.nrl_ranges[plan_index];
    r.nrl_b = r.nrl_ranges[plan_index + 1];
  }
  void set_road_lane_plan_batch(const std::vector<uint>& road_indices,
                                const std::vector<uint>& plan_indices) {
    for (int i = 0, s = road_indices.size(); i < s; ++i) {
      set_road_lane_plan(road_indices[i], plan_indices[i]);
    }
  }
  void set_vehicle_route(int index, const vec<int>& route, int end_lane_id,
                         float end_s) {
    S.person.SetRoute(index, route, end_lane_id, end_s);
  }
  auto debug_vehicle_info() {
    auto* out = new vec<double>();
    out->reserve(S.person.M->veh_cnt * 5);
    for (auto& p : S.person.persons) {
      if (p.runtime.status == PersonStatus::DRIVING) {
        out->push_back(p.id);
        out->push_back(p.runtime.lane->id);
        out->push_back(p.runtime.s);
        out->push_back(p.runtime.acc);
        out->push_back((int)p._reason);
      }
    }
    return asarray(out);
  }
  auto debug_vehicle_full(uint id) {
#define ID(x) ((x) ? int((x)->id) : -1)
    auto& p = *S.person.person_map.at(id);
    std::vector<uint> route;
    p.route.veh->route.Save(route);
    return std::make_tuple(
        std::make_tuple(p.start_time),
        std::make_tuple(p.runtime.speed, p.runtime.acc, int(p._reason),
                        int(p._reason_detail),
                        p.node.front ? int(p.node.front->self->id) : -1,
                        ID(p.next_lane),
                        p.next_lane ? p.next_lane->max_speed : -1,
                        p.next_lane ? int(p.next_lane->light_state) : -1),
        std::make_tuple((int)p.snapshot.status, ID(p.snapshot.lane),
                        p.snapshot.s, p.snapshot.shadow_s, p.snapshot.speed,
                        p.snapshot.is_lane_changing, ID(p.snapshot.shadow_lane),
                        p.snapshot.lc_length, p.snapshot.lc_length,
                        p.snapshot.lc_total_length,
                        p.snapshot.lc_complete_length),
        std::make_tuple(
            (int)p.runtime.status, ID(p.runtime.lane), p.runtime.s,
            p.runtime.shadow_s, p.runtime.speed, p.runtime.is_lane_changing,
            ID(p.runtime.shadow_lane), p.runtime.lc_length, p.runtime.lc_length,
            p.runtime.lc_total_length, p.runtime.lc_complete_length),
        std::make_tuple(route, p.route_index, p.route_lc_offset));
  }
  auto debug_lane_info() {
    vec<vec<int>> out;
    out.reserve(S.lane.lanes.size);
    for (auto& l : S.lane.lanes) {
      vec<int> vs;
      auto* p = l.veh_head;
      while (p) {
        vs.push_back(p->self->id * (p->is_shadow ? -1 : 1));
        p = p->next;
      }
      out.push_back(std::move(vs));
    }
    return out;
  }
  void next_step(uint n) {
    // 中间的步骤就没有必要更新输出，节省时间
    S.enable_api_output = false;
    for (int i = 1; i < n; ++i) {
      S.Step();
    }
    if (n) {
      S.enable_api_output = true;
      S.Step();
    }
  }
  auto save() { return S.Save(); }
  void load(size_t checkpoint_id) { S.Load(checkpoint_id); }
};

void set_device(int device_id) { CUCHECK(cudaSetDevice(device_id)); }
int get_device() {
  int device;
  CUCHECK(cudaGetDevice(&device));
  return device;
}

PYBIND11_MODULE(_simulet, m) {
  py::class_<Engine>(m, "Engine")
      .def_readonly_static("__version__", &VER)
      .def(py::init<const std::string&, const std::string&, uint, float, int,
                    int, int, bool, bool, int, float, float, uint, float, uint,
                    uint, float, uint>(),
           "map_file"_a, "agent_file"_a, "start_step"_a, "step_interval"_a = 1,
           "seed"_a = 43, "verbose_level"_a = 0, "agent_limit"_a = -1,
           "disable_aoi_out_control"_a = false, "disable_junction"_a = false,
           "junction_blocking_count"_a = -1, "junction_yellow_time"_a = 3,
           "phase_pressure_coeff"_a = 1.5, "lane_change"_a = 1,
           "mobil_lc_forbidden_distance"_a = 15,
           "lane_veh_add_buffer_size"_a = 300,
           "lane_veh_remove_buffer_size"_a = 300, "speed_stat_interval"_a = 0,
           "device"_a = 0, no_gil())
      .def("next_step", &Engine::next_step, "n"_a = 1, no_gil())
      .def("get_current_time", &Engine::get_current_time, no_gil())
      .def("get_vehicle_count", &Engine::get_vehicle_count, no_gil())
      .def("get_vehicle_ids", &Engine::get_vehicle_ids, no_gil())
      .def("get_vehicle_lanes", &Engine::get_vehicle_lanes, no_gil())
      .def("get_vehicle_speeds", &Engine::get_vehicle_speeds, no_gil())
      .def("get_vehicle_distances", &Engine::get_vehicle_distances, no_gil())
      .def("get_vehicle_total_distances", &Engine::get_vehicle_total_distances,
           no_gil())
      .def("get_vehicle_positions", &Engine::get_vehicle_positions, no_gil())
      .def("get_vehicle_positions_all", &Engine::get_vehicle_positions_all,
           no_gil())
      .def("get_vehicle_id_positions", &Engine::get_vehicle_id_positions,
           no_gil())
      .def("get_vehicle_statuses", &Engine::get_vehicle_statuses, no_gil())
      .def("get_vehicle_raw_statuses", &Engine::get_vehicle_raw_statuses,
           no_gil())
      .def("get_vehicle_avds", &Engine::get_vehicle_avds, no_gil())
      .def("get_running_vehicle_count", &Engine::get_running_vehicle_count,
           no_gil())
      .def("get_finished_vehicle_count", &Engine::get_finished_vehicle_count,
           no_gil())
      .def("get_vehicle_traveling_or_departure_times",
           &Engine::get_vehicle_traveling_or_departure_times, no_gil())
      .def("get_finished_vehicle_average_traveling_time",
           &Engine::get_finished_vehicle_average_traveling_time, no_gil())
      .def("get_running_vehicle_average_traveling_time",
           &Engine::get_running_vehicle_average_traveling_time, no_gil())
      .def("get_departed_vehicle_average_traveling_time",
           &Engine::get_departed_vehicle_average_traveling_time, no_gil())
      .def("get_lane_count", &Engine::get_lane_count, no_gil())
      .def("get_lane_ids", &Engine::get_lane_ids, no_gil())
      .def("get_lane_statuses", &Engine::get_lane_statuses, no_gil())
      .def("get_lane_geoms", &Engine::get_lane_geoms, no_gil())
      .def("get_lane_lengths", &Engine::get_lane_lengths, no_gil())
      .def("get_lane_vehicle_counts", &Engine::get_lane_vehicle_counts,
           no_gil())
      .def("get_lane_waiting_vehicle_counts",
           &Engine::get_lane_waiting_vehicle_counts, "speed_threshold"_a = 0.1,
           no_gil())
      .def("get_lane_waiting_at_end_vehicle_counts",
           &Engine::get_lane_waiting_at_end_vehicle_counts,
           "speed_threshold"_a = 0.1, "distance_to_end"_a = 100, no_gil())
      .def("get_lane_average_vehicle_speed",
           &Engine::get_lane_average_vehicle_speed, "lane_index"_a, no_gil())
      .def("get_junction_ids", &Engine::get_junction_ids, no_gil())
      .def("get_junction_count", &Engine::get_junction_count, no_gil())
      .def("get_junction_lanes", &Engine::get_junction_lanes, no_gil())
      .def("get_junction_inout_lanes", &Engine::get_junction_inout_lanes,
           no_gil())
      .def("get_junction_phase_lanes", &Engine::get_junction_phase_lanes,
           no_gil())
      .def("get_junction_phase_ids", &Engine::get_junction_phase_ids, no_gil())
      .def("get_junction_phase_counts", &Engine::get_junction_phase_counts,
           no_gil())
      .def("get_junction_dynamic_roads", &Engine::get_junction_dynamic_roads,
           no_gil())
      .def("get_road_count", &Engine::get_road_count, no_gil())
      .def("get_road_lane_plans", &Engine::get_road_lane_plans, no_gil())
      .def("get_road_lane_plan_index", &Engine::get_road_lane_plan_index,
           no_gil())
      .def("get_road_vehicle_counts", &Engine::get_road_vehicle_counts,
           no_gil())
      .def("get_road_waiting_vehicle_counts",
           &Engine::get_road_waiting_vehicle_counts, "speed_threshold"_a = 0.1,
           no_gil())
      .def("get_road_average_vehicle_speed",
           &Engine::get_road_average_vehicle_speed, "road_index"_a, no_gil())
      .def("set_vehicle_enable", &Engine::set_vehicle_enable, "vehicle_index"_a,
           "enable"_a, no_gil())
      .def("set_vehicle_enable_batch", &Engine::set_vehicle_enable_batch,
           "person_indices"_a, "enable"_a, no_gil())
      .def("set_lane_restriction", &Engine::set_lane_restriction,
           "lane_index"_a, "flag"_a, no_gil())
      .def("set_lane_restriction_batch", &Engine::set_lane_restriction_batch,
           "lane_indices"_a, "flags"_a, no_gil())
      .def("set_lane_max_speed", &Engine::set_lane_max_speed, "lane_index"_a,
           "max_speed"_a, no_gil())
      .def("set_lane_max_speed_batch", &Engine::set_lane_max_speed_batch,
           "lane_indices"_a, "max_speeds"_a, no_gil())
      .def("set_tl_policy", &Engine::set_tl_policy, "junction_index"_a,
           "policy"_a, no_gil())
      .def("set_tl_policy_batch", &Engine::set_tl_policy_batch,
           "junction_indices"_a, "policy"_a, no_gil())
      .def("set_tl_duration", &Engine::set_tl_duration, "junction_index"_a,
           "duration"_a, no_gil())
      .def("set_tl_duration_batch", &Engine::set_tl_duration_batch,
           "junction_indices"_a, "duration"_a, no_gil())
      .def("set_tl_phase", &Engine::set_tl_phase, "junction_index"_a,
           "phase_index"_a, no_gil())
      .def("set_tl_phase_batch", &Engine::set_tl_phase_batch,
           "junction_indices"_a, "phase_indices"_a, no_gil())
      .def("set_road_lane_plan", &Engine::set_road_lane_plan, "road_index"_a,
           "plan_index"_a, no_gil())
      .def("set_road_lane_plan_batch", &Engine::set_road_lane_plan_batch,
           "road_indices"_a, "plan_indices"_a, no_gil())
      .def("set_vehicle_route", &Engine::set_vehicle_route, "vehicle_index"_a,
           "route"_a, "end_lane_id"_a, "end_s"_a, no_gil())
      .def("debug_vehicle_info", &Engine::debug_vehicle_info, no_gil())
      .def("debug_vehicle_full", &Engine::debug_vehicle_full, "id"_a, no_gil())
      .def("debug_lane_info", &Engine::debug_lane_info, no_gil())
      .def("make_checkpoint", &Engine::save, no_gil())
      .def("restore_checkpoint", &Engine::load, "checkpoint_id"_a, no_gil());
  m.attr("__version__") = VER;
  m.def("set_device", set_device, "device_id"_a, no_gil());
  m.def("get_device", get_device, no_gil());
}

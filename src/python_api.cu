#include <cuda.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include "moss.cuh"
#include "utils/color_print.h"
#include "utils/macro.h"

namespace py = pybind11;
using namespace py::literals;
using namespace moss;
template <class T>
using vec = std::vector<T>;
using std::pair;
using no_gil = py::call_guard<py::gil_scoped_release>;

template <typename T>
inline py::array_t<typename T::value_type> asarray(T* ptr) {
  py::gil_scoped_acquire _;
  return py::array(ptr->size(), ptr->data(), py::capsule(ptr, [](void* p) {
                     delete reinterpret_cast<T*>(p);
                   }));
}
template <class T>
vec<T>& remove_duplicate(vec<T>& arr) {
  std::sort(arr.begin(), arr.end());
  arr.erase(std::unique(arr.begin(), arr.end()), arr.end());
  return arr;
}

class Engine {
 private:
  Moss S;

 public:
  Engine(const std::string& name, const std::string& map_file,
         const std::string& person_file, uint start_step, float step_interval,
         int seed, int verbose_level, int person_limit,
         float junction_yellow_time, float phase_pressure_coeff,
         float speed_stat_interval, const std::string& output_dir,
         float out_xmin, float out_ymin, float out_xmax, float out_ymax,
         uint device)
      : S() {
    auto config = Config{
        .map_file = map_file,
        .person_file = person_file,
        .person_limit = uint(person_limit),
        .speed_stat_interval = speed_stat_interval,
        .start_step = start_step,
        .total_step = 1 << 30,
        .step_interval = step_interval,
        .seed = seed,
        .output = {.enable = !output_dir.empty(),
                   .dir = output_dir,
                   .tl_file_duration = 60},
        .x_min = out_xmin,
        .y_min = out_ymin,
        .x_max = out_xmax,
        .y_max = out_ymax,
        .verbose_level = verbose_level,
        .n_workers = 0,
        .junction_yellow_time = junction_yellow_time,
        .phase_pressure_coeff = phase_pressure_coeff,
        .device = device,
    };
    S.Init(name, config);
  }

  // 获取当前步
  // Get the current step
  int get_current_step() { return S.step; }

  // 获取当前时间
  // Get the current time
  float get_current_time() { return S.time; }

  // get persons' all information as numpy arrays
  auto fetch_persons() {
    Info("Call fetch_persons");
    // define
    auto* ids = new vec<int>();
    auto* enable = new vec<int8_t>();
    auto* statuses = new vec<int>();
    auto* lane_ids = new vec<int>();
    auto* lane_parent_ids = new vec<int>();
    auto* ss = new vec<float>();
    auto* aoi_ids = new vec<int>();
    auto* vs = new vec<float>();
    auto* shadow_lane_ids = new vec<int>();
    auto* shadow_ss = new vec<float>();
    auto* lc_yaws = new vec<float>();
    auto* lc_completed_ratios = new vec<float>();
    auto* is_forwards = new vec<int8_t>();
    auto* xs = new vec<float>();
    auto* ys = new vec<float>();
    auto* dirs = new vec<float>();
    auto* schedule_indexs = new vec<int>();
    auto* trip_indexs = new vec<int>();
    auto* departure_times = new vec<float>();
    auto* traveling_times = new vec<float>();
    auto* total_distances = new vec<float>();
    // reserve space
    ids->reserve(S.person.persons.size);
    enable->reserve(S.person.persons.size);
    statuses->reserve(S.person.persons.size);
    lane_ids->reserve(S.person.persons.size);
    lane_parent_ids->reserve(S.person.persons.size);
    ss->reserve(S.person.persons.size);
    aoi_ids->reserve(S.person.persons.size);
    vs->reserve(S.person.persons.size);
    shadow_lane_ids->reserve(S.person.persons.size);
    shadow_ss->reserve(S.person.persons.size);
    lc_yaws->reserve(S.person.persons.size);
    lc_completed_ratios->reserve(S.person.persons.size);
    is_forwards->reserve(S.person.persons.size);
    xs->reserve(S.person.persons.size);
    ys->reserve(S.person.persons.size);
    dirs->reserve(S.person.persons.size);
    schedule_indexs->reserve(S.person.persons.size);
    trip_indexs->reserve(S.person.persons.size);
    departure_times->reserve(S.person.persons.size);
    traveling_times->reserve(S.person.persons.size);
    total_distances->reserve(S.person.persons.size);
    Info("Call fetch_persons: reserve space");
    // copy
    for (auto& p : S.person.persons) {
      ids->push_back(int(p.id));
      enable->push_back(int8_t(p.enable));
      statuses->push_back(int(p.runtime.status));
      if (p.runtime.lane) {
        lane_ids->push_back(p.runtime.lane->id);
        if (p.runtime.lane->parent_is_road) {
          lane_parent_ids->push_back(int(p.runtime.lane->parent_road->id));
        } else {
          lane_parent_ids->push_back(int(p.runtime.lane->parent_junction->id));
        }
      } else {
        lane_ids->push_back(-1);
        lane_parent_ids->push_back(-1);
      }
      ss->push_back(p.runtime.s);
      aoi_ids->push_back(p.runtime.aoi ? int(p.runtime.aoi->id) : -1);
      vs->push_back(p.runtime.v);
      shadow_lane_ids->push_back(
          p.runtime.shadow_lane ? int(p.runtime.shadow_lane->id) : -1);
      shadow_ss->push_back(p.runtime.shadow_s);
      lc_yaws->push_back(p.runtime.lc_yaw);
      lc_completed_ratios->push_back(p.runtime.lc_completed_ratio);
      is_forwards->push_back(p.runtime.is_forward);
      xs->push_back(p.runtime.x);
      ys->push_back(p.runtime.y);
      dirs->push_back(p.runtime.dir);
      schedule_indexs->push_back(p.schedule_index);
      trip_indexs->push_back(p.trip_index);
      departure_times->push_back(p.departure_time);
      traveling_times->push_back(p.traveling_time);
      total_distances->push_back(p.total_distance);
    }

    return std::make_tuple(
        asarray(ids), asarray(enable), asarray(statuses), asarray(lane_ids),
        asarray(lane_parent_ids), asarray(ss), asarray(aoi_ids), asarray(vs),
        asarray(shadow_lane_ids), asarray(shadow_ss), asarray(lc_yaws),
        asarray(lc_completed_ratios), asarray(is_forwards), asarray(xs),
        asarray(ys), asarray(dirs), asarray(schedule_indexs),
        asarray(trip_indexs), asarray(departure_times),
        asarray(traveling_times), asarray(total_distances));
  }

  auto fetch_lanes() {
    Info("Call fetch_lanes");
    // define
    auto* ids = new vec<int>();
    auto* statuses = new vec<int8_t>();
    auto* v_avgs = new vec<float>();

    // reserve space
    ids->reserve(S.lane.lanes.size);
    statuses->reserve(S.lane.lanes.size);
    v_avgs->reserve(S.lane.lanes.size);

    Info("Call fetch_lanes: reserve space");

    // copy
    for (auto& l : S.lane.lanes) {
      ids->push_back(int(l.id));
      // 0: green, 1: yellow, 2: red, 3: restriction
      statuses->push_back((l.restriction | l.in_restriction)               ? 3
                          : l.light_state == LightState::LIGHT_STATE_GREEN ? 0
                          : l.light_state == LightState::LIGHT_STATE_YELLOW
                              ? 1
                              : 2);
      v_avgs->push_back(l.v_avg);
    }

    return std::make_tuple(asarray(ids), asarray(statuses), asarray(v_avgs));
  }

  // 获取道路id列表
  // Get the list of road ids
  auto get_road_ids() {
    Info("Call get_road_ids");
    auto* out = new vec<int>();
    out->reserve(S.road.roads.size);
    for (auto& r : S.road.roads) {
      out->push_back(r.id);
    }
    Info("Call get_road_ids: return out");
    return asarray(out);
  }

  // 按顺序获取路口id列表
  // Get the list of junction ids in order
  auto get_junction_ids() {
    Info("Call get_junction_ids");
    auto* out = new vec<int>();
    out->reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      out->push_back(j.id);
    }
    Info("Call get_junction_ids: return out");
    return asarray(out);
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
  // Get the current phase id of the junction
  auto get_junction_phase_ids() {
    auto* out = new vec<int>();
    out->reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      out->push_back(j.tl.ok ? j.tl.phase_index : (unsigned)-1);
    }
    return asarray(out);
  }

  // 获取路口的相位数
  // Get the number of phases of the junction
  auto get_junction_phase_counts() {
    auto* out = new vec<int>();
    out->reserve(S.junction.junctions.size);
    for (auto& j : S.junction.junctions) {
      out->push_back(j.tl.phases.size);
    }
    return asarray(out);
  }

  // 获取与路口相连的动态道路index
  // Get the dynamic road index connected to the junction
  auto get_junction_dynamic_roads() {
    vec<vec<int>> out(S.junction.junctions.size);
    for (auto& r : S.road.roads) {
      if (r.nrl_ranges) {
        out[r.lanes[0]->successor->parent_junction->index].push_back(r.index);
      }
    }
    return out;
  }

  // 获取动态道路的车道方案
  // Get the lane plan of the dynamic road
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
        o.emplace_back(r.next_road_lane_groups[j].offset1,
                       r.next_road_lane_groups[j].offset2);
      }
    }
    return out;
  }

  // 获取动态道路的当前车道方案id
  // Get the current lane plan id of the dynamic road
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

  // 获取用于输出的人的信息
  // get person information for output ([(id, status, parent_id, x, y, dir),
  // ...], [x, ...], [y, ...])
  auto get_output_persons() {
    vec<std::tuple<int, int, int, float, float>> out;
    auto xs = new vec<float>;
    auto ys = new vec<float>;
    out.reserve(S.person.persons.size);
    xs->reserve(S.person.persons.size);
    ys->reserve(S.person.persons.size);
    for (auto& p : S.person.persons) {
      if (p.runtime.status == PersonStatus::DRIVING ||
          p.runtime.status == PersonStatus::WALKING) {
        if (S.config.x_min <= p.runtime.x && p.runtime.x <= S.config.x_max &&
            S.config.y_min <= p.runtime.y && p.runtime.y <= S.config.y_max) {
          out.emplace_back(p.id, int(p.runtime.status),
                           p.runtime.lane ? p.runtime.lane->id : unsigned(-1),
                           p.runtime.dir, p.runtime.v);
          xs->push_back(p.runtime.x);
          ys->push_back(p.runtime.y);
        }
      }
    }
    return std::make_tuple(out, asarray(xs), asarray(ys));
  }

  // 获取用于输出的信号灯信息
  // get traffic light information for output ([(id, state), ...], [x, ...], [y,
  // ...])
  auto get_output_tls() {
    vec<std::tuple<int, int>> out;
    auto xs = new vec<float>;
    auto ys = new vec<float>;
    out.reserve(S.lane.output_lanes.size);
    xs->reserve(S.lane.output_lanes.size);
    ys->reserve(S.lane.output_lanes.size);
    for (auto* l : S.lane.output_lanes) {
      out.emplace_back(l->id, l->light_state);
      xs->push_back(l->center_x);
      ys->push_back(l->center_y);
    }
    return std::make_tuple(out, asarray(xs), asarray(ys));
  }

  // 获取用于输出的路况信息
  // get lane information for output [(id, level, v), ...]
  auto get_output_roads() {
    vec<std::tuple<int, int, float>> out;
    out.reserve(S.road.roads.size);
    for (auto& r : S.road.roads) {
      out.emplace_back(r.id, int(r.status), r.v_avg);
    }
    return out;
  }

  // set person enable
  void set_person_enable(uint person_index, bool enable) {
    if (person_index >= S.person.persons.size) {
      throw std::out_of_range("Person index out of range.");
    }
    S.person.persons[person_index].enable = enable;
  }

  // set person enable in batch
  void set_person_enable_batch(const std::vector<uint>& person_indices,
                               const std::vector<bool>& enable) {
    for (int i = 0, s = person_indices.size(); i < s; ++i) {
      set_person_enable(person_indices[i], enable[i]);
    }
  }

  // set traffic light policy
  void set_tl_policy(uint junction_index, uint policy) {
    if (junction_index >= S.junction.junctions.size) {
      throw std::out_of_range("Junction index out of range.");
    }
    if (policy > 3) {
      throw std::out_of_range("Invalid policy id.");
    }
    S.junction.junctions[junction_index].tl_policy = (TlPolicy)policy;
  }

  // set traffic light policy in batch
  void set_tl_policy_batch(const std::vector<uint>& junction_indices,
                           uint policy) {
    for (auto& i : junction_indices) {
      set_tl_policy(i, policy);
    }
  }

  // set traffic light duration
  void set_tl_duration(uint junction_index, int duration) {
    if (junction_index >= S.junction.junctions.size) {
      throw std::out_of_range("Junction index out of range.");
    }
    S.junction.junctions[junction_index].phase_time = duration;
  }

  // set traffic light duration in batch
  void set_tl_duration_batch(const std::vector<uint>& junction_indices,
                             uint policy) {
    for (auto& i : junction_indices) {
      set_tl_duration(i, policy);
    }
  }

  // set traffic light phase
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

  // set traffic light phase in batch
  void set_tl_phase_batch(const std::vector<uint>& junction_indices,
                          const std::vector<uint>& phase_indices) {
    for (int i = 0, s = junction_indices.size(); i < s; ++i) {
      set_tl_phase(junction_indices[i], phase_indices[i]);
    }
  }

  // set lane restriction
  void set_lane_restriction(uint lane_index, bool flag) {
    if (lane_index >= S.lane.lanes.size) {
      throw std::out_of_range("Lane index out of range.");
    }
    S.lane.lanes[lane_index].restriction = flag;
  }

  // set lane restriction in batch
  void set_lane_restriction_batch(const std::vector<uint>& lane_indices,
                                  const std::vector<bool>& flags) {
    for (int i = 0, s = lane_indices.size(); i < s; ++i) {
      set_lane_restriction(lane_indices[i], flags[i]);
    }
  }

  // set lane max speed
  void set_lane_max_speed(uint lane_index, float max_speed) {
    if (lane_index >= S.lane.lanes.size) {
      throw std::out_of_range("Lane index out of range.");
    }
    S.lane.lanes[lane_index].max_speed = max_speed;
  }

  // set lane max speed in batch
  void set_lane_max_speed_batch(const std::vector<uint>& lane_indices,
                                const std::vector<float>& max_speeds) {
    for (int i = 0, s = lane_indices.size(); i < s; ++i) {
      set_lane_max_speed(lane_indices[i], max_speeds[i]);
    }
  }

  // 设置动态道路的车道方案
  // set road lane plan
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

  // set vehicle route, route is a list of road ids
  void set_vehicle_route(uint person_id, const std::vector<uint>& route) {
    auto p = S.person.At(person_id);
    p->SetVehicleRoute(&S, route);
  }

  void next_step(uint n) {
    // // 中间的步骤就没有必要更新输出，节省时间
    for (int i = 1; i < n; ++i) {
      S.Step();
    }
    if (n) {
      S.Step();
    }
  }

  int save() { return S.Save(); }

  void restore(int checkpoint_id) { S.Restore(checkpoint_id); }
};

void set_device(int device_id) { CUCHECK(cudaSetDevice(device_id)); }
int get_device() {
  int device;
  CUCHECK(cudaGetDevice(&device));
  return device;
}

PYBIND11_MODULE(_moss, m) {
  py::class_<Engine>(m, "Engine")
      .def_readonly_static("__version__", &VER)
      .def(py::init<const std::string&, const std::string&, const std::string&,
                    uint, float, int, int, int, float, float, float,
                    const std::string&, float, float, float, float, uint>(),
           "name"_a, "map_file"_a, "person_file"_a, "start_step"_a,
           "step_interval"_a = 1, "seed"_a = 43, "verbose_level"_a = 0,
           "person_limit"_a = -1, "junction_yellow_time"_a = 3,
           "phase_pressure_coeff"_a = 1.5, "speed_stat_interval"_a = 0,
           "output_dir"_a = "", "out_xmin"_a = 0, "out_ymin"_a = 0,
           "out_xmax"_a = 0, "out_ymax"_a = 0, "device"_a = 0, no_gil())
      .def("next_step", &Engine::next_step, "n"_a = 1, no_gil())
      .def("get_current_step", &Engine::get_current_step, no_gil())
      .def("get_current_time", &Engine::get_current_time, no_gil())
      .def("fetch_persons", &Engine::fetch_persons, no_gil())
      .def("fetch_lanes", &Engine::fetch_lanes, no_gil())
      .def("get_road_ids", &Engine::get_road_ids, no_gil())
      .def("get_junction_ids", &Engine::get_junction_ids, no_gil())
      .def("get_junction_phase_lanes", &Engine::get_junction_phase_lanes,
           no_gil())
      .def("get_junction_phase_ids", &Engine::get_junction_phase_ids, no_gil())
      .def("get_junction_phase_counts", &Engine::get_junction_phase_counts,
           no_gil())
      .def("get_junction_dynamic_roads", &Engine::get_junction_dynamic_roads,
           no_gil())
      .def("get_road_lane_plans", &Engine::get_road_lane_plans, no_gil())
      .def("get_road_lane_plan_index", &Engine::get_road_lane_plan_index,
           no_gil())
      .def("get_output_persons", &Engine::get_output_persons, no_gil())
      .def("get_output_tls", &Engine::get_output_tls, no_gil())
      .def("get_output_roads", &Engine::get_output_roads, no_gil())
      .def("set_person_enable", &Engine::set_person_enable, "person_index"_a,
           "enable"_a, no_gil())
      .def("set_person_enable_batch", &Engine::set_person_enable_batch,
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
      .def("set_vehicle_route", &Engine::set_vehicle_route, "person_id"_a,
           "route"_a, no_gil())
      .def("make_checkpoint", &Engine::save, no_gil())
      .def("restore_checkpoint", &Engine::restore, "checkpoint_id"_a, no_gil());
  m.attr("__version__") = VER;
  m.def("set_device", set_device, "device_id"_a, no_gil());
  m.def("get_device", get_device, no_gil());
}

#include <cmath>
#include <cuda.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include "mem/mem.cuh"
#include "output.cuh"
#include "protos.h"
#include "rpc/routing.cuh"
#include "simulet.cuh"
#include "utils/color_print.h"
#include "utils/debug.cuh"
#include "utils/profile.h"
#include "utils/timer.h"

namespace simulet {

std::atomic_int simulet_id;

size_t Simulet::Save() {
  auto* p = new Checkpoint;
  checkpoints[(size_t)p] = p;
  p->step = step;
  p->time = time;
  p->veh_cnt = person.M->veh_cnt;
  p->ped_cnt = person.M->ped_cnt;
  p->crowd_cnt = person.M->crowd_cnt;
  p->finished_cnt = person.M->finished_cnt;
  p->traveling_time = person.M->traveling_time;
  p->finished_traveling_time = person.M->finished_traveling_time;
  lane.Save(p->lanes);
  road.Save(p->roads);
  junction.Save(p->junctions);
  aoi.Save(p->aois);
  person.Save(p->persons);
  person.veh_lane.Save(p->veh_lane);
  person.veh_speed.Save(p->veh_speed);
  person.veh_distance.Save(p->veh_distance);
  person.veh_total_distance.Save(p->veh_total_distance);
  return (size_t)p;
}

void Simulet::Load(size_t id) {
  if (checkpoints.find(id) == checkpoints.end()) {
    throw std::range_error("Specified checkpoint does not exist.");
  }
  auto* p = checkpoints.at(id);
  step = p->step;
  time = p->time;
  person.M->veh_cnt = p->veh_cnt;
  person.M->ped_cnt = p->ped_cnt;
  person.M->crowd_cnt = p->crowd_cnt;
  person.M->finished_cnt = p->finished_cnt;
  person.M->traveling_time = p->traveling_time;
  person.M->finished_traveling_time = p->finished_traveling_time;
  lane.Load(p->lanes);
  road.Load(p->roads);
  junction.Load(p->junctions);
  aoi.Load(p->aois);
  person.Load(p->persons);
  person.veh_lane.Load(p->veh_lane);
  person.veh_speed.Load(p->veh_speed);
  person.veh_distance.Load(p->veh_distance);
  person.veh_total_distance.Load(p->veh_total_distance);
}

void Simulet::Init(const Config& config_) {
  id = simulet_id++;
  config = config_;
  color_print::enable = verbose = config.verbose_level;
  // 关闭输出缓冲
  setbuf(stdout, NULL);
  // 初始化设备
  CUCHECK(cudaSetDevice(config.device));
  CUCHECK(cudaFree(0));
  CUCHECK(cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount,
                                 config.device));
  Info("Using GPU ", config.device, " with ", sm_count, " SMs");
  // 初始化内存池
  {
    Tag _("Mem Init", 0);
    mem = MemManager::New(20_GiB, config.device, verbose);
  }
  if (config.n_workers == 0) {
    config.n_workers = get_nprocs();
  }
  Info("Using ", config.n_workers, " CPUs");

  // 初始化全局变量
  step = config.start_step;
  time = step * config.step_interval;
  seed = config.seed = config.seed == 0 ? Time() : config.seed;
  Info("Using seed=", config.seed);
  if (config.disable_aoi_out_control) {
    Warn("Disable AOI out control");
    aoi.out_control = false;
  }
  Info("Junction: ", config.enable_junction ? "True" : "False");
  Info("Junction yellow time: ", config.junction_yellow_time);
  Info("Junction blocking count: ", config.junction_blocking_count);
  if (config.output_type == "disable") {
    output.option = output::Option::DISABLE;
    Info("Output: False");
  } else {
    Info("Output: ", config.output_file);
    if (config.output_type == "agent") {
      output.option = output::Option::AGENT;
      Info("OutputType: Agent");
    } else if (config.output_type == "lane") {
      output.option = output::Option::LANE;
      Info("OutputType: Lane");
    } else {
      Fatal("Unknown output_type: ", config.output_type);
    }
  }
  output.X_MIN = config.x_min;
  output.Y_MIN = config.y_min;
  output.X_MAX = config.x_max;
  output.Y_MAX = config.y_max;
  road.k_status = config.speed_stat_interval <= 0
                      ? 0
                      : exp(-config.step_interval / config.speed_stat_interval);

  // 读入数据文件
  PbMap map;
  PbAgents agents;
  {
    Tag _("Reading files", 1);
    Info("Reading map...");
    std::fstream file;
    file.open(config.map_file, std::ios::in | std::ios::binary);
    if (!file) {
      Fatal("Cannot open file: ", config.map_file);
    }
    if (!map.ParseFromIstream(&file)) {
      Fatal("Failed to parse map file.");
    }
    file.close();
    Info("Reading agents...");
    file.open(config.agent_file, std::ios::in | std::ios::binary);
    if (!file) {
      Fatal("Cannot open file: ", config.agent_file);
    }
    if (!agents.ParseFromIstream(&file)) {
      Fatal("Failed to parse agent file.");
    }
    file.close();
    Info("Lane: ", map.lanes_size());
    Info("Road: ", map.roads_size());
    Info("Junction: ", map.junctions_size());
    Info("AOI: ", map.aois_size());
    Info("Person: ", agents.agents_size());
  }

  // 更新上限
  config.agent_limit = min(config.agent_limit, agents.agents_size());

  // 初始化各类模拟对象
  {
    mem->PreferCPU();
    Tag _("Entity Init", 2);
    Info("+ Lane");
    lane.Init(this, map);
    Info("P<", lane.g_prepare1, ",", lane.b_prepare1, ">");
    mem->PrintUsage();
    Info("+ Road");
    road.Init(this, map);
    mem->PrintUsage();
    Info("+ Junction");
    junction.Init(this, map);
    Info("P<", junction.g_prepare, ",", junction.b_prepare, ">");
    Info("U<", junction.g_update, ",", junction.b_update, ">");
    Info("Junction output: ", lane.output_lanes.size);
    mem->PrintUsage();
    Info("+ AOI");
    aoi.Init(this, map);
    Info("P<", aoi.g_prepare, ",", aoi.b_prepare, ">");
    Info("U<", aoi.g_update, ",", aoi.b_update, ">");
    mem->PrintUsage();
    if (!is_python_api) {
      Info("+ Post");
      routing.Init(this, config.routing_url, config.n_workers);
      mem->PrintUsage();
    }
    lane.InitSizes(this);
    Info("+ Person");
    person.Init(this, agents, config.agent_limit);
    Info("P<", person.g_prepare, ",", person.b_prepare, ">");
    Info("U<", person.g_update, ",", person.b_update, ">");
    mem->PrintUsage();
    mem->PreferGPU();
  }
  Info("done.");
  CHECK;
  color_print::enable = verbose = config.verbose_level > 1;
  // 输出
  if (output.option != output::Option::DISABLE) {
    output.Init(this, config.output_file);
    if (output.option == output::Option::AGENT) {
      for (auto& l : lane.lanes) {
        auto& x = l.center_x;
        auto& y = l.center_y;
        l.need_output = !l.parent_is_road && output.X_MIN <= x &&
                        x <= output.X_MAX && output.Y_MIN <= y &&
                        y <= output.Y_MAX;
      }
    }
  }
}

void Simulet::Step() {
  uint64_t t = Time();
  Tag _(fmt::format("Step: {} Veh: {} Ped: {}", step - config.start_step,
                    person.M->veh_cnt, person.M->ped_cnt)
            .c_str(),
        7);
#if STUCK_MONITOR
  Info("Step: ", global::step - global::START_STEP, " (", global::step, ") ",
       fmt::format("{:.2f}", 1e6 / max(1, t - last_t)),
       "Hz Veh:", person.veh_cnt, " Stuck:", person.stuck_cnt,
       " Ped:", person.ped_cnt, " Crowd:", person.crowd_cnt);
#else
  Info("[", id, "] Step: ", step - config.start_step, " (", step, ") ",
       fmt::format("{:.2f}", 1e6 / max(1, t - last_step_t)),
       "Hz Veh:", person.M->veh_cnt, " Ped:", person.M->ped_cnt,
       " Crowd:", person.M->crowd_cnt);
#endif
  last_step_t = t;
  // 准备
  {
    Tag _("Prepare", 3);
    person.PrepareAsync();
    aoi.PrepareAsync();
    junction.PrepareAsync();
    // 需要等待位置更新后才能更新链表
    CUCHECK(cudaStreamSynchronize(person.stream));
    lane.PrepareAsync();
    CUCHECK(cudaDeviceSynchronize());
    CHECK_ERROR;
  }
  if (!is_python_api) {
    Tag _("Routing Prepare", 4);
    routing.Prepare();
  }
  {
    Tag _("Wait output", 8);
    output.Wait();
  }
  // 更新
  {
    Tag _("Update", 5);
    lane.UpdateAsync();
    person.UpdateAsync();
    aoi.UpdateAsync();
    junction.UpdateAsync();
    CUCHECK(cudaDeviceSynchronize());
    CHECK_ERROR;
  }
  // 输出
  {
    Tag _("Output start", 9);
    output.Start();
  }
  // 处理导航请求
  if (!is_python_api) {
    Tag _(fmt::format("Routing Requests: {}", routing.d_post->size).c_str(), 6);
    routing.ProcessRequests();
    CHECK;
  }
  ++step;
  time = step * config.step_interval;
}

void Simulet::Run() {
  CUCHECK(cudaSetDevice(config.device));
  for (uint i = 0; i < config.total_step; ++i) {
    Step();
  }
  Info("Simulation complete");
  output.Stop();
  routing.StopWorkers();
}

void Simulet::Stop() {
  output.Stop();
  ++step;
  time = step * config.step_interval;
}

};  // namespace simulet

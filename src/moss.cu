#include <cmath>
#include <cuda.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include "mem/mem.cuh"
#include "moss.cuh"
#include "protos.h"
#include "utils/color_print.h"
#include "utils/debug.cuh"
#include "utils/profile.h"
#include "utils/timer.h"

namespace moss {

std::atomic_int moss_id;

void Moss::Init(const std::string& name, const Config& config_) {
  id = moss_id++;
  config = config_;
  if (!config.output.enable) {
    config.output.dir = "";
  }
  step_output.Init(config.output.dir, name, config.map_file,
                   config.output.tl_file_duration);
  color_print::enable = verbose = config.verbose_level > 0;
  // disable output buffer for better performance
  setbuf(stdout, NULL);
  // init cuda device
  CUCHECK(cudaSetDevice(config.device));
  CUCHECK(cudaFree(0));  // just check it
  CUCHECK(cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount,
                                 config.device));
  Info("Using GPU ", config.device, " with ", sm_count, " SMs");
  // init memory pool
  { mem = new MemManager{config.device, verbose}; }
  // init multi-thread
  if (config.n_workers == 0) {
    config.n_workers = get_nprocs();
  }
  Info("Using ", config.n_workers, " CPUs");

  // init global variables
  step = config.start_step;
  time = step * config.step_interval;
  seed = config.seed = config.seed == 0 ? Time() : config.seed;
  Info("Using seed=", config.seed);
  Info("Junction yellow time: ", config.junction_yellow_time);
  // init road k parameter for speed statistics
  road.k_status = exp(-config.step_interval / config.speed_stat_interval);

  // read map and persons
  PbMap map;
  PbPersons persons;
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
    Info("Reading persons...");
    file.open(config.person_file, std::ios::in | std::ios::binary);
    if (!file) {
      Fatal("Cannot open file: ", config.person_file);
    }
    if (!persons.ParseFromIstream(&file)) {
      Fatal("Failed to parse agent file.");
    }
    file.close();
    map_west = map.header().west();
    map_east = map.header().east();
    map_south = map.header().south();
    map_north = map.header().north();
    Info("Lane: ", map.lanes_size());
    Info("Road: ", map.roads_size());
    Info("Junction: ", map.junctions_size());
    Info("AOI: ", map.aois_size());
    Info("Person: ", persons.persons_size());
  }

  // init simulation entities
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
    mem->PrintUsage();
    lane.InitSizes(this);
    Info("+ Person");
    person.Init(this, persons, config.person_limit);
    Info("P<", person.g_prepare, ",", person.b_prepare, ">");
    Info("U<", person.g_update, ",", person.b_update, ">");
    mem->PrintUsage();
    mem->PreferGPU();
  }
  Info("done.");
  CHECK;
  color_print::enable = verbose = config.verbose_level > 1;
}

void Moss::Step() {
  uint64_t t = Time();
  Tag _(fmt::format("Step: {}", step - config.start_step).c_str(), 7);
  Info("[", id, "] Step: ", step - config.start_step, " (", step, ") ",
       fmt::format("{:.2f}", 1e6 / max(1ul, t - last_step_t)),
       "Hz Sleep:", person.M->status_cnts[0],
       " Walk:", person.M->status_cnts[1], " Drive:", person.M->status_cnts[2],
       " Finished:", person.M->status_cnts[3]);
  last_step_t = t;
  // Prepare stage
  {
    Tag _("Prepare", 3);
    person.PrepareAsync();
    junction.PrepareAsync();
    // Need to wait for the position update (person) before updating the lane
    // list
    lane.PrepareAsync(person.stream);
    lane.PrepareOutputAsync(junction.stream);
    CUCHECK(cudaDeviceSynchronize());
    CHECK_ERROR;
  }
  // Update stage
  {
    Tag _("Update", 5);
    lane.UpdateAsync();
    person.UpdateAsync();
    junction.UpdateAsync();
    // write output into file
    step_output.Write(time, person.outputs, lane.outputs);
    CUCHECK(cudaDeviceSynchronize());
    CHECK_ERROR;
  }
  ++step;
  time = step * config.step_interval;
}

void Moss::Run() {
  CUCHECK(cudaSetDevice(config.device));
  for (uint i = 0; i < config.total_step; ++i) {
    Step();
  }
  step_output.Close();
  Info("Simulation complete");
}

void Moss::Stop() {
  ++step;
  time = step * config.step_interval;
  step_output.Close();
}

void Moss::Close() {
  delete mem;
  Info("Memory pool released");
  Info("Moss closed");
}

int Moss::Save() {
  Checkpoint checkpoint;
  checkpoint.step = step;
  checkpoint.mem_save_id = mem->Save();
  checkpoints.push_back(checkpoint);
  return checkpoints.size() - 1;
}

void Moss::Restore(int id) {
  if (id < 0 || id >= checkpoints.size()) {
    Fatal("Restore: invalid snapshot id: ", id);
  }
  step = checkpoints[id].step;
  time = step * config.step_interval;
  mem->Restore(checkpoints[id].mem_save_id);
}

};  // namespace moss

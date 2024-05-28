#include <unistd.h>
#include "argparse/argparse.hpp"
#include "entity/junction/junction.cuh"
#include "simulet.cuh"
#include "utils/barrier.h"
#include "utils/macro.h"
#include "utils/timer.h"
#include "yaml-cpp/node/parse.h"
#include "yaml-cpp/yaml.h"

int main(int argc, char** argv) {
  argparse::ArgumentParser parser("Simulet", VER);
  parser.add_argument("-c", "--config").required().help("path to config file");
  parser.add_argument("-q", "--quiet")
      .default_value(false)
      .implicit_value(true);
  try {
    parser.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << parser;
    return 1;
  }
  if (!parser.get<bool>("quiet")) {
    printf(RED("Simulet " VER "\n"));
  }

  auto cfg = YAML::LoadFile(parser.get<std::string>("config"));
  simulet::Simulet s{};
  s.is_python_api = true;
  auto t_init = Time();
  s.Init({
      .map_file = cfg["map_file"].as<std::string>(),
      .agent_file = cfg["agent_file"].as<std::string>(),
      .agent_limit = (uint)cfg["agent_limit"].as<int>(),
      .output_file = cfg["output_file"].as<std::string>(),
      .output_type = cfg["output_type"].as<std::string>(),
      .road_status_interval = cfg["road_status_interval"].as<float>(),
      .start_step = cfg["start_step"].as<uint>(),
      .total_step = cfg["total_step"].as<uint>(),
      .step_interval = cfg["step_interval"].as<float>(),
      .routing_url = cfg["routing_url"].as<std::string>(),
      .pre_routing = cfg["pre_routing"].as<bool>(),
      .enable_aoi_indoor = cfg["enable_aoi_indoor"].as<bool>(),
      .enable_junction = cfg["enable_junction"].as<bool>(),
      .seed = cfg["seed"].as<int>(),
      .x_min = cfg["x_min"].as<float>(),
      .y_min = cfg["y_min"].as<float>(),
      .x_max = cfg["x_max"].as<float>(),
      .y_max = cfg["y_max"].as<float>(),
      .verbose_level = cfg["verbose_level"].as<int>(),
      .disable_aoi_out_control = cfg["disable_aoi_out_control"].as<bool>(),
      .n_workers = 0,
      .junction_blocking_count = (uint)cfg["junction_blocking_count"].as<int>(),
      .junction_yellow_time = cfg["junction_yellow_time"].as<float>(),
      .phase_pressure_coeff = cfg["phase_pressure_coeff"].as<float>(),
      .lane_change_algorithm = (uint)cfg["lane_change_algorithm"].as<int>(),
      .mobil_lc_forbidden_distance = 15,
      .lane_veh_add_buffer_size = (uint)cfg["lane_add_buffer_size"].as<int>(),
      .lane_veh_remove_buffer_size =
          (uint)cfg["lane_remove_buffer_size"].as<int>(),
      .device = (uint)cfg["device"].as<int>(),
  });
  t_init = Time() - t_init;
  auto tl_policy = (simulet::TlPolicy)cfg["junction_tl_policy"].as<int>();
  auto tl_duration = cfg["junction_tl_duration"].as<float>();
  for (auto& j : s.junction.junctions) {
    j.tl_policy = tl_policy;
    j.phase_time = tl_duration;
  }
  auto t_run = Time();
  s.Run();
  t_run = Time() - t_run;
  printf("Time: %.3f + %.3f\n", double(t_init) * 1e-6, double(t_run) * 1e-6);
}

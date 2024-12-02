#include <unistd.h>
#include "argparse/argparse.hpp"
#include "entity/junction/junction.cuh"
#include "moss.cuh"
#include "utils/color_print.h"
#include "utils/macro.h"
#include "utils/timer.h"
#include "yaml-cpp/node/parse.h"
#include "yaml-cpp/yaml.h"

// main function
int main(int argc, char** argv) {
  // parse arguments
  // -c, --config: path to config file
  // -q, --quiet: quiet mode
  argparse::ArgumentParser parser("Moss", VER);
  parser.add_argument("-n", "--name").required().help("name of the simulation");
  parser.add_argument("-c", "--config").required().help("path to config file");
  parser.add_argument("--gpu").default_value(0).help("GPU device ID");
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
  // show version, which is defined in CMakeLists.txt
  if (!parser.get<bool>("quiet")) {
    printf(RED("Moss " VER "\n"));
  }

  // read config file
  auto name = parser.get<std::string>("name");
  auto cfg = YAML::LoadFile(parser.get<std::string>("config"));
  moss::Moss s{};
  auto t_init = Time();
  std::string map_file;
  if (cfg["map_file"]) {
    map_file = cfg["map_file"].as<std::string>();
  } else {
    Fatal("`map_file` is required in the config file");
  }
  std::string person_file;
  if (cfg["person_file"]) {
    person_file = cfg["person_file"].as<std::string>();
  } else {
    Fatal("`person_file` is required in the config file");
  }
  uint person_limit = uint(-1);  // default: INF
  if (cfg["person_limit"]) {
    person_limit = (uint)cfg["person_limit"].as<int>();
  }
  moss::OutputConfig output_cfg = {
      .enable = false, .dir = "", .tl_file_duration = 10};
  if (cfg["output"]) {
    if (cfg["output"]["enable"]) {
      output_cfg.enable = cfg["output"]["enable"].as<bool>();
    }
    if (output_cfg.enable) {
      if (cfg["output"]["dir"]) {
        output_cfg.dir = cfg["output"]["dir"].as<std::string>();
      } else {
        Fatal("`output.dir` is required in the config file");
      }
      if (cfg["output"]["tl_file_duration"]) {
        output_cfg.tl_file_duration =
            cfg["output"]["tl_file_duration"].as<float>();
        if (output_cfg.tl_file_duration <= 0) {
          Fatal("`output.tl_file_duration` should be positive");
        }
      }
    }
  }
  float speed_stat_interval = 300;  // default: 300
  if (cfg["speed_stat_interval"]) {
    speed_stat_interval = cfg["speed_stat_interval"].as<float>();
    assert(speed_stat_interval > 0);
  }
  uint start_step, total_step;
  if (cfg["start_step"]) {
    start_step = cfg["start_step"].as<uint>();
  } else {
    Fatal("`start_step` is required in the config file");
  }
  if (cfg["total_step"]) {
    total_step = cfg["total_step"].as<uint>();
  } else {
    Fatal("`total_step` is required in the config file");
  }
  float step_interval = 1;  // default: 1
  if (cfg["step_interval"]) {
    step_interval = cfg["step_interval"].as<float>();
  }
  int seed = 43;  // default: 43
  if (cfg["seed"]) {
    seed = cfg["seed"].as<int>();
  }
  float x_min = -1e999, y_min = -1e999, x_max = 1e999, y_max = 1e999;
  if (cfg["x_min"]) {
    x_min = cfg["x_min"].as<float>();
  }
  if (cfg["y_min"]) {
    y_min = cfg["y_min"].as<float>();
  }
  if (cfg["x_max"]) {
    x_max = cfg["x_max"].as<float>();
  }
  if (cfg["y_max"]) {
    y_max = cfg["y_max"].as<float>();
  }
  int verbose_level = 2;  // default: 2
  if (cfg["verbose_level"]) {
    verbose_level = cfg["verbose_level"].as<int>();
  }
  float junction_yellow_time = 0;  // default: 0
  if (cfg["junction_yellow_time"]) {
    junction_yellow_time = cfg["junction_yellow_time"].as<float>();
  }
  float phase_pressure_coeff = 1.5;  // default: 1.5
  if (cfg["phase_pressure_coeff"]) {
    phase_pressure_coeff = cfg["phase_pressure_coeff"].as<float>();
  }
  float device_mem = 0;
  if (cfg["device_mem"]) {
    device_mem = cfg["device_mem"].as<float>();
  }

  uint device = 0;
  if (cfg["gpu"]) {
    device = cfg["gpu"].as<int>();
  }

  s.Init(name, {
                   .map_file = map_file,
                   .person_file = person_file,
                   .person_limit = person_limit,
                   .speed_stat_interval = speed_stat_interval,
                   .start_step = start_step,
                   .total_step = total_step,
                   .step_interval = step_interval,
                   .seed = seed,
                   .output = output_cfg,
                   .x_min = x_min,
                   .y_min = y_min,
                   .x_max = x_max,
                   .y_max = y_max,
                   .verbose_level = verbose_level,
                   .n_workers = 0,
                   .junction_yellow_time = junction_yellow_time,
                   .phase_pressure_coeff = phase_pressure_coeff,
                   .device = device,
                   .device_mem = device_mem,
               });
  t_init = Time() - t_init;
  // set junction traffic light policy
  auto tl_policy = (moss::TlPolicy)cfg["junction_tl_policy"].as<int>();
  auto tl_duration = cfg["junction_tl_duration"].as<float>();
  for (auto& j : s.junction.junctions) {
    j.tl_policy = tl_policy;
    j.phase_time = tl_duration;
  }
  // run simulation
  auto t_run = Time();
  s.Run();
  t_run = Time() - t_run;
  printf("Time: %.3f + %.3f\n", double(t_init) * 1e-6, double(t_run) * 1e-6);
}

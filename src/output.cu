#include <string>
#include "containers/vector.cuh"
#include "entity/person/person.cuh"
#include "entity/road/road.cuh"
#include "output.cuh"
#include "simulet.cuh"
#include "utils/color_print.h"

namespace simulet::output {

template <class T>
void write(std::ofstream& s, const T& x) {
  s.write((char*)&x, sizeof(T));
}

void Data::worker() {
  while (true) {
    barrier.wait();
    if (stop) {
      return;
    }
    if (option == Option::AGENT) {
      AgentOutput o = {
          .type = output::AgentOutputType::CONTROL_INFO,
      };
      o.step = S->step;
      file.write((char*)&o, sizeof(AgentOutput));
      if (M->agent_output.size) {
        file.write((char*)M->agent_output.data,
                   sizeof(AgentOutput) * M->agent_output.size);
        M->agent_output.size = 0;
      }
    } else if (option == Option::LANE) {
      write<uint>(file, S->step);
      file.write(M->road_output.data, M->road_output.size);
    }
    barrier.wait();
    file.flush();
    if (stop) {
      return;
    }
  }
}

void Data::Init(Simulet* S, const std::string& filename) {
  this->S = S;
  M = S->mem->MValueZero<MData>();
  file.open(filename, std::ios::out | std::ios::binary);
  if (!file) {
    Fatal("Cannot open output file: ", filename);
  }
  barrier.set(2);
  worker_t = std::thread(&Data::worker, this);
  write<uint>(file, S->config.start_step);
  write<uint>(file, S->config.total_step);
  write<float>(file, S->config.step_interval);
  if (option == Option::AGENT) {
    write<uint>(file, S->person.persons.size);
    M->agent_output = {};
    M->agent_output.mem = S->mem;
    M->agent_output.Reserve(10000);
  } else if (option == Option::LANE) {
    write<uint>(file, S->road.roads.size);
    for (auto& r : S->road.roads) {
      write<uint>(file, r.id);
    }
    M->road_output = {};
    M->road_output.New(S->mem, S->road.roads.size);
  }
}

void Data::Start() {
  if (option == Option::DISABLE) return;
  barrier.wait();
}

void Data::Wait() {
  if (option == Option::DISABLE) return;
  barrier.wait();
}

void Data::Stop() {
  if (option == Option::DISABLE) return;
  stop = true;
  barrier.wait();
  worker_t.join();
}

}  // namespace simulet::output

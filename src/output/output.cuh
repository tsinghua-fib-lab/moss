#ifndef SRC_OUTPUT_OUTPUT_CUH_
#define SRC_OUTPUT_OUTPUT_CUH_

#include <filesystem>
#include <string>
#include "avro/DataFile.hh"
#include "containers/array.cuh"
#include "output/person.h"
#include "output/tl.h"

namespace fs = std::filesystem;

namespace moss {

class StepOutput {
 public:
  StepOutput();
  ~StepOutput();
  /*
    tl_file_duration: the duration of each traffic light output file. If it is
    10.0, the first file will be 0.0-10.0.avro, the second file will be
    10.0-20.0.avro, and so on.
    We will use an increamental way to write the output files for saving
    storage. As a trade-off, the loading time will be longer.
  */
  void Init(const std::string& output_dir, const std::string& name,
            const std::string& map_path,
            float tl_file_duration);
  void Write(float t, const MArrZ<PersonOutput>& person_outputs,
             const MArrZ<TlOutput>& tl_outputs);
  void FlushTl();
  void Close();

 private:
  bool _enable = false;
  fs::path _output_dir;
  avro::ValidSchema _person_schema;
  avro::ValidSchema _tl_schema;
  float _tl_file_duration;
  float _tl_file_start_t = -1e999;
  float _last_tl_file_t = -1e999;
  avro::DataFileWriter<moss::TlOutput>* _tl_writer = nullptr;
  std::vector<TlOutput> _last_tl_outputs;
};

}  // namespace moss

#endif

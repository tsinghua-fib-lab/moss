#include <fstream>
#include <sstream>
#include "avro/Compiler.hh"
#include "avro/ValidSchema.hh"
#include "entity/person/person.cuh"
#include "output/output.cuh"
#include "output/schema.h"
#include "utils/color_print.h"

namespace moss {

StepOutput::StepOutput() {
  {
    std::istringstream is(PERSON_SCHEMA);
    avro::compileJsonSchema(is, _person_schema);
  }
  {
    std::istringstream is(TL_SCHEMA);
    avro::compileJsonSchema(is, _tl_schema);
  }
}

StepOutput::~StepOutput() {}

void StepOutput::Init(const std::string& output_dir, const std::string& name,
                      const std::string& map_path, float tl_file_duration) {
  _tl_file_duration = tl_file_duration;
  if (output_dir.empty()) {
    Info("Info: `output_dir` is not set, no output will be generated");
    return;
  }
  _enable = true;
  _output_dir = fs::path(output_dir) / name;
  fs::create_directories(_output_dir);
  // if the directory is not empty, clear it
  if (!fs::is_empty(_output_dir)) {
    Warn("Output directory is not empty: ", _output_dir.c_str());
    fs::remove_all(_output_dir);
  }
  fs::create_directories(_output_dir / "vehicle");
  fs::create_directories(_output_dir / "pedestrian");
  fs::create_directories(_output_dir / "tl");
  // copy map file to output directory
  fs::copy_file(map_path, _output_dir / "map.pb",
                fs::copy_options::overwrite_existing);
  // write an empty YAML file to indicate the output directory is valid
  std::ofstream project_file(_output_dir / "moss.yml", std::ios::out);
  project_file.close();
}

void StepOutput::Write(float t, const MArrZ<PersonOutput>& person_outputs,
                       const MArrZ<TlOutput>& tl_outputs) {
  if (!_enable) {
    return;
  }
  // person: vehicle + pedestrian
  // filename schema: {output_dir}/person_{t}.avro
  // t: float, only keep 2 decimal places
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << t;
  std::string vehicle_filename = _output_dir / "vehicle" / (ss.str() + ".avro");
  std::string ped_filename = _output_dir / "pedestrian" / (ss.str() + ".avro");
  avro::DataFileWriter<moss::PersonOutput> veh_writer(vehicle_filename.c_str(),
                                                      _person_schema);
  avro::DataFileWriter<moss::PersonOutput> ped_writer(ped_filename.c_str(),
                                                      _person_schema);
  for (const auto& person_output : person_outputs) {
    switch (person_output.status) {
      case int(PersonStatus::DRIVING): {
        veh_writer.write(person_output);
      } break;
      case int(PersonStatus::WALKING): {
        ped_writer.write(person_output);
      } break;
      default:
        // Handle unexpected status if necessary
        break;
    }
  }
  veh_writer.close();
  ped_writer.close();

  // traffic light: incrementally write

  if (t >= _tl_file_start_t + _tl_file_duration) {
    // close the previous writer
    FlushTl();
  }
  if (_tl_writer == nullptr) {
    // reset the start time
    _tl_file_start_t = t;
    // open a new writer
    _tl_writer = new avro::DataFileWriter<moss::TlOutput>(
        (_output_dir / "tl" / "tmp.avro").c_str(), _tl_schema);
    // full write
    for (const auto& tl_output : tl_outputs) {
      _tl_writer->write(tl_output);
    }
    tl_outputs.Save(_last_tl_outputs);
  } else {
    // incremental write
    assert(tl_outputs.size == uint(_last_tl_outputs.size()));
    int incremental_count = 0;
    for (uint i = 0; i < tl_outputs.size; ++i) {
      assert(tl_outputs[i].id == _last_tl_outputs[i].id);
      if (tl_outputs[i].state != _last_tl_outputs[i].state) {
        _tl_writer->write(tl_outputs[i]);
        _last_tl_outputs[i] = tl_outputs[i];
        ++incremental_count;
      }
    }
    if (incremental_count == 0) {
      // add a record to indicate the time
      assert(tl_outputs.size > 0);
      _tl_writer->write(tl_outputs[0]);
    }
    _tl_writer->flush();
  }
  _last_tl_file_t = t;
}

void StepOutput::FlushTl() {
  if (!_enable) {
    return;
  }
  if (_tl_writer != nullptr) {
    _tl_writer->close();
    delete _tl_writer;
    _tl_writer = nullptr;
    _last_tl_outputs.clear();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << _tl_file_start_t;
    ss << "-";
    ss << std::fixed << std::setprecision(2) << _last_tl_file_t;
    std::string tl_filename = _output_dir / "tl" / (ss.str() + ".avro");
    fs::rename(_output_dir / "tl" / "tmp.avro", tl_filename);
  }
}

void StepOutput::Close() {
  if (!_enable) {
    return;
  }
  FlushTl();
}

}  // namespace moss

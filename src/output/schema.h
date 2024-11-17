#ifndef SRC_OUTPUT_SCHEMA_H_
#define SRC_OUTPUT_SCHEMA_H_

#include <string>

namespace moss {

const std::string PERSON_SCHEMA = R"({
  "type": "record",
  "name": "PersonOutput",
  "fields": [
    {
      "name": "t",
      "type": "float"
    },
    {
      "name": "id",
      "type": "int"
    },
    {
      "name": "x",
      "type": "float"
    },
    {
      "name": "y",
      "type": "float"
    },
    {
      "name": "v",
      "type": "float"
    },
    {
      "name": "parent_id",
      "type": "int"
    },
    {
      "name": "direction",
      "type": "float"
    },
    {
      "name": "model",
      "type": "string"
    },
    {
      "name": "status",
      "type": "int"
    }
  ]
})";

const std::string TL_SCHEMA = R"({
  "type": "record",
  "name": "TlOutput",
  "fields": [
    {
      "name": "t",
      "type": "float"
    },
    {
      "name": "id",
      "type": "int"
    },
    {
      "name": "state",
      "type": "int"
    }
  ]
})";

}  // namespace moss

#endif

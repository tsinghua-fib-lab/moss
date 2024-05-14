#ifndef SRC_UTILS_PROFILE_H_
#define SRC_UTILS_PROFILE_H_
#include "nvToolsExt.h"

const uint32_t colors[] = {0xff1f77b4, 0xffaec7e8, 0xffff7f0e, 0xffffbb78,
                           0xff2ca02c, 0xff98df8a, 0xffd62728, 0xffff9896,
                           0xff9467bd, 0xffc5b0d5, 0xff8c564b, 0xffc49c94,
                           0xffe377c2, 0xfff7b6d2, 0xff7f7f7f, 0xffc7c7c7,
                           0xffbcbd22, 0xffdbdb8d, 0xff17becf, 0xff9edae5};
const int num_colors = sizeof(colors) / sizeof(uint32_t);

#define PUSH_RANGE(name, cid)                          \
  {                                                    \
    int color_id = cid;                                \
    color_id = color_id % num_colors;                  \
    nvtxEventAttributes_t eventAttrib = {0};           \
    eventAttrib.version = NVTX_VERSION;                \
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;  \
    eventAttrib.colorType = NVTX_COLOR_ARGB;           \
    eventAttrib.color = colors[color_id];              \
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII; \
    eventAttrib.message.ascii = name;                  \
    nvtxRangePushEx(&eventAttrib);                     \
  }
#define POP_RANGE nvtxRangePop();

class Tag {
 public:
  Tag(const char* name, int cid) {
    int color_id = cid;
    color_id = color_id % num_colors;
    nvtxEventAttributes_t eventAttrib = {0};
    eventAttrib.version = NVTX_VERSION;
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    eventAttrib.colorType = NVTX_COLOR_ARGB;
    eventAttrib.color = colors[color_id];
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
    eventAttrib.message.ascii = name;
    nvtxRangePushEx(&eventAttrib);
  }
  ~Tag() { nvtxRangePop(); }
};

#endif

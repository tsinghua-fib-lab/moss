/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This code was generated by avrogencpp 1.12.0. Do not edit.*/

#ifndef SRC_OUTPUT_TL_H_476768912_H
#define SRC_OUTPUT_TL_H_476768912_H


#include <sstream>
#include <any>
#include "avro/Specific.hh"
#include "avro/Encoder.hh"
#include "avro/Decoder.hh"

namespace moss {
struct TlOutput {
    float t;
    int32_t id;
    int32_t state;
    TlOutput() :
        t(float()),
        id(int32_t()),
        state(int32_t())
        { }
};

}
namespace avro {
template<> struct codec_traits<moss::TlOutput> {
    static void encode(Encoder& e, const moss::TlOutput& v) {
        avro::encode(e, v.t);
        avro::encode(e, v.id);
        avro::encode(e, v.state);
    }
    static void decode(Decoder& d, moss::TlOutput& v) {
        if (avro::ResolvingDecoder *rd =
            dynamic_cast<avro::ResolvingDecoder *>(&d)) {
            const std::vector<size_t> fo = rd->fieldOrder();
            for (std::vector<size_t>::const_iterator it = fo.begin();
                it != fo.end(); ++it) {
                switch (*it) {
                case 0:
                    avro::decode(d, v.t);
                    break;
                case 1:
                    avro::decode(d, v.id);
                    break;
                case 2:
                    avro::decode(d, v.state);
                    break;
                default:
                    break;
                }
            }
        } else {
            avro::decode(d, v.t);
            avro::decode(d, v.id);
            avro::decode(d, v.state);
        }
    }
};

}
#endif

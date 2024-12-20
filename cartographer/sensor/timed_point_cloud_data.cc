/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/sensor/timed_point_cloud_data.h"

#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data) {
  proto::TimedPointCloudData proto;
  proto.set_timestamp(common::ToUniversal(timed_point_cloud_data.time));
  *proto.mutable_origin() = transform::ToProto(timed_point_cloud_data.origin);
  proto.mutable_point_data()->Reserve(timed_point_cloud_data.ranges.size());
  for (const TimedRangefinderPoint& range : timed_point_cloud_data.ranges) {
    *proto.add_point_data() = ToProto(range);
  }
  for (const float intensity : timed_point_cloud_data.intensities) {
    proto.add_intensities(intensity);
  }
  proto.set_closure_weight_factor(timed_point_cloud_data.closure_weight_factor);
  for (const LineFeature& line_feature : timed_point_cloud_data.line_features) {
    proto::LineFeature* line_feature_proto = proto.add_line_features();
    *line_feature_proto->mutable_start() = transform::ToProto(line_feature.start);
    *line_feature_proto->mutable_end() = transform::ToProto(line_feature.end);
  }
  return proto;
}

TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto) {
  CHECK(proto.intensities().size() == 0 ||
        proto.intensities().size() == proto.point_data().size());
  TimedPointCloud timed_point_cloud;
  if (proto.point_data().size() > 0) {
    timed_point_cloud.reserve(proto.point_data().size());
    for (const auto& timed_point_proto : proto.point_data()) {
      timed_point_cloud.push_back(FromProto(timed_point_proto));
    }
  } else {
    timed_point_cloud.reserve(proto.point_data_legacy().size());
    for (const auto& timed_point_proto : proto.point_data_legacy()) {
      const Eigen::Vector4f timed_point = transform::ToEigen(timed_point_proto);
      timed_point_cloud.push_back({timed_point.head<3>(), timed_point[3]});
    }
  }
  std::vector<LineFeature> line_features;
  for (const auto& line_feature_proto : proto.line_features()) {
    line_features.push_back({transform::ToEigen(line_feature_proto.start()),
                             transform::ToEigen(line_feature_proto.end())});
  }
  return TimedPointCloudData{common::FromUniversal(proto.timestamp()),
                             transform::ToEigen(proto.origin()),
                             timed_point_cloud,
                             std::vector<float>(proto.intensities().begin(),
                                                proto.intensities().end()),
                              proto.closure_weight_factor(),
                              line_features};
}

}  // namespace sensor
}  // namespace cartographer

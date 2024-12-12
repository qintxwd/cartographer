/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
      range_data.closure_weight_factor,
      TransformLineFeatures(range_data.line_features, transform)
  };
}

RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin,
                   CropPointCloud(range_data.returns, min_z, max_z),
                   CropPointCloud(range_data.misses, min_z, max_z), range_data.closure_weight_factor,
                   range_data.line_features};
}

proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  proto.mutable_returns()->Reserve(range_data.returns.size());
  for (const RangefinderPoint& point : range_data.returns) {
    *proto.add_returns() = ToProto(point);
  }
  proto.mutable_misses()->Reserve(range_data.misses.size());
  for (const RangefinderPoint& point : range_data.misses) {
    *proto.add_misses() = ToProto(point);
  }
  proto.set_closure_weight_factor(range_data.closure_weight_factor);
  for (const LineFeature& line_feature : range_data.line_features) {
    proto::LineFeature* line_feature_proto = proto.add_line_features();
    *line_feature_proto->mutable_start() = transform::ToProto(line_feature.start);
    *line_feature_proto->mutable_end() = transform::ToProto(line_feature.end);
  }
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  std::vector<RangefinderPoint> returns;
  if (proto.returns_size() > 0) {
    returns.reserve(proto.returns().size());
    for (const auto& point_proto : proto.returns()) {
      returns.push_back(FromProto(point_proto));
    }
  } else {
    returns.reserve(proto.returns_legacy().size());
    for (const auto& point_proto : proto.returns_legacy()) {
      returns.push_back({transform::ToEigen(point_proto)});
    }
  }
  std::vector<RangefinderPoint> misses;
  if (proto.misses_size() > 0) {
    misses.reserve(proto.misses().size());
    for (const auto& point_proto : proto.misses()) {
      misses.push_back(FromProto(point_proto));
    }
  } else {
    misses.reserve(proto.misses_legacy().size());
    for (const auto& point_proto : proto.misses_legacy()) {
      misses.push_back({transform::ToEigen(point_proto)});
    }
  }
  std::vector<LineFeature> line_features;
  for (const auto& line_feature_proto : proto.line_features()) {
    line_features.push_back({transform::ToEigen(line_feature_proto.start()),
                             transform::ToEigen(line_feature_proto.end())});
  }
  return RangeData{transform::ToEigen(proto.origin()), PointCloud(returns),
                   PointCloud(misses), proto.closure_weight_factor(), line_features};
}

}  // namespace sensor
}  // namespace cartographer

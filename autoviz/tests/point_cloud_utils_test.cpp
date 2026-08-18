/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/point_cloud_utils.hpp"

#include <cstring>
#include <string>

#include <gtest/gtest.h>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>

namespace autoviz {
namespace display {
namespace {

using Cloud = automsgs::msgs::sensor_msgs::PointCloud2;
using Field = automsgs::msgs::sensor_msgs::PointField;

void AddField(Cloud* cloud, const std::string& name, uint32_t offset,
              int datatype) {
  Field* field = cloud->add_fields();
  field->set_name(name);
  field->set_offset(offset);
  field->set_datatype(static_cast<Field::DataType>(datatype));
  field->set_count(1);
}

TEST(PointCloudUtils, ReadsNamedIntensityChannel) {
  Cloud cloud;
  cloud.set_width(2);
  cloud.set_height(1);
  cloud.set_point_step(16);
  cloud.set_row_step(32);
  cloud.set_is_dense(true);
  AddField(&cloud, "x", 0, Field::FLOAT32);
  AddField(&cloud, "y", 4, Field::FLOAT32);
  AddField(&cloud, "z", 8, Field::FLOAT32);
  AddField(&cloud, "reflectivity", 12, Field::FLOAT32);
  float data[8] = {1.f, 2.f, 3.f, 9.f, 4.f, 5.f, 6.f, 11.f};
  cloud.set_data(std::string(reinterpret_cast<const char*>(data), sizeof(data)));

  const ParsedPointCloud parsed = parsePointCloud2(cloud, 1, "reflectivity");
  ASSERT_EQ(parsed.xs.size(), 2u);
  ASSERT_EQ(parsed.intensities.size(), 2u);
  EXPECT_FLOAT_EQ(parsed.intensities[0], 9.f);
  EXPECT_FLOAT_EQ(parsed.intensities[1], 11.f);
}

TEST(PointCloudUtils, ScalarColorInvertsRainbow) {
  const QColor low = colorFromScalar(0.f, 0.f, 1.f, true, false, Qt::black, Qt::white);
  const QColor high_inverted =
      colorFromScalar(1.f, 0.f, 1.f, true, true, Qt::black, Qt::white);
  EXPECT_EQ(low.red(), high_inverted.red());
  EXPECT_EQ(low.green(), high_inverted.green());
  EXPECT_EQ(low.blue(), high_inverted.blue());
}

}  // namespace
}  // namespace display
}  // namespace autoviz

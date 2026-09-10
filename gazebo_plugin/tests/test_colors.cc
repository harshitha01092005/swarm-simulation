// Pure protocol/material tests. They remain active in Release builds.
#include "FleetColors.hh"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace
{
void Check(bool condition, const char *message)
{
  if (!condition)
  {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

gz::msgs::Pose_V Colors()
{
  gz::msgs::Pose_V message;
  auto *pose = message.add_pose();
  auto *frame = pose->mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value("world");
  auto *child = pose->mutable_header()->add_data();
  child->set_key("child_frame_id");
  child->add_value("drone_002");
  pose->mutable_position()->set_x(1.0);
  pose->mutable_position()->set_y(0.25);
  pose->mutable_position()->set_z(0.0);
  pose->mutable_orientation()->set_w(1.0);
  return message;
}
}  // namespace

int main()
{
  using drone_swarm::ColorsMatchActive;
  using drone_swarm::ParseColorBatch;
  const auto valid = Colors();
  const auto parsed = ParseColorBatch(valid, 5);
  Check(parsed.has_value(), "Jazzy TF header identity and valid RGB accepted");
  Check(parsed->at(1)->red == 1 && parsed->at(1)->green == 0.25 && parsed->at(1)->blue == 0,
        "RGB channel ordering and values preserved");
  Check(ColorsMatchActive(*parsed, {false, true, false, false, false}),
        "full active fleet identity set accepted");
  Check(!ColorsMatchActive(*parsed, {true, true, false, false, false}),
        "omitted active drone rejects whole color batch");
  Check(!ColorsMatchActive(*parsed, {false, false, false, false, false}),
        "inactive drone color rejects whole color batch");
  Check(!ColorsMatchActive(*parsed, {false, true}), "capacity mismatch rejected");
  const auto empty = ParseColorBatch(gz::msgs::Pose_V(), 5);
  Check(empty && ColorsMatchActive(*empty, {false, false, false, false, false}),
        "empty colors accepted only for empty active fleet");
  Check(!ParseColorBatch(valid, 0) && !ParseColorBatch(valid, 401), "capacity bounds enforced");
  Check(!ParseColorBatch(valid, 1), "out-of-capacity identity rejected");
  auto duplicate = valid;
  *duplicate.add_pose() = valid.pose(0);
  Check(!ParseColorBatch(duplicate, 5), "duplicate RGB identity rejected atomically");
  auto ambiguous = valid;
  *ambiguous.mutable_pose(0)->mutable_header()->add_data() = valid.pose(0).header().data(1);
  Check(!ParseColorBatch(ambiguous, 5), "ambiguous child header rejected");
  auto wrongFrame = valid;
  wrongFrame.mutable_pose(0)->mutable_header()->mutable_data(0)->set_value(0, "map");
  Check(!ParseColorBatch(wrongFrame, 5), "wrong frame rejected");
  auto conflicting = valid;
  conflicting.mutable_pose(0)->set_name("ground");
  Check(!ParseColorBatch(conflicting, 5), "conflicting visual identity rejected");
  auto noPosition = valid;
  noPosition.mutable_pose(0)->clear_position();
  Check(!ParseColorBatch(noPosition, 5), "missing RGB payload rejected");
  auto noRotation = valid;
  noRotation.mutable_pose(0)->clear_orientation();
  Check(!ParseColorBatch(noRotation, 5), "missing protocol rotation rejected");
  for (const double value : {-0.01, 1.01,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    auto invalid = valid;
    invalid.mutable_pose(0)->mutable_position()->set_x(value);
    Check(!ParseColorBatch(invalid, 5), "invalid red rejected");
    invalid = valid;
    invalid.mutable_pose(0)->mutable_position()->set_y(value);
    Check(!ParseColorBatch(invalid, 5), "invalid green rejected");
    invalid = valid;
    invalid.mutable_pose(0)->mutable_position()->set_z(value);
    Check(!ParseColorBatch(invalid, 5), "invalid blue rejected");
  }
  for (const double value : {0.0, 2.0, std::numeric_limits<double>::quiet_NaN()})
  {
    auto invalid = valid;
    invalid.mutable_pose(0)->mutable_orientation()->set_w(value);
    Check(!ParseColorBatch(invalid, 5), "non-identity rotation rejected");
  }
  auto badRotation = valid;
  badRotation.mutable_pose(0)->mutable_orientation()->set_z(0.1);
  Check(!ParseColorBatch(badRotation, 5), "nonzero rotation rejected");

  gz::msgs::Pose_V large;
  for (std::size_t i = 1; i <= 400; ++i)
  {
    auto *pose = large.add_pose();
    *pose = valid.pose(0);
    std::ostringstream name;
    name << "drone_" << std::setfill('0') << std::setw(3) << i;
    pose->mutable_header()->mutable_data(1)->set_value(0, name.str());
  }
  const auto fleet = ParseColorBatch(large, 400);
  Check(fleet && ColorsMatchActive(*fleet, std::vector<bool>(400, true)),
        "full 400-drone color batch accepted");
  const auto visual = drone_swarm::ColorVisualCommand(42, *parsed->at(1));
  Check(visual.id() == 42, "resolved visual entity ID preserved");
  for (const auto &color : {visual.material().ambient(), visual.material().diffuse(),
                            visual.material().emissive()})
    Check(color.r() == 1 && color.g() == 0.25 && color.b() == 0 && color.a() == 1,
          "ambient/diffuse/emissive command channels and opaque alpha preserved");
  Check(visual.material().specular().r() == 0 && visual.material().specular().a() == 1,
        "show light specular does not illuminate a black LED");
  std::cout << "Fleet RGB protocol and material command validation passed.\n";
  return EXIT_SUCCESS;
}

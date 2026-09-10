// Run through CTest in the Gazebo build environment. Do not use assert():
// its checks disappear from the Release builds used for the Docker image.
#include "FleetProtocol.hh"

#include <cstdlib>
#include <iostream>
#include <limits>

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

gz::msgs::Pose_V ValidBatch()
{
  gz::msgs::Pose_V message;
  auto *pose = message.add_pose();
  auto *frame = pose->mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value("world");
  auto *child = pose->mutable_header()->add_data();
  child->set_key("child_frame_id");
  child->add_value("drone_002");
  pose->mutable_position()->set_x(2.0);
  pose->mutable_position()->set_y(-3.0);
  pose->mutable_position()->set_z(4.0);
  pose->mutable_orientation()->set_w(1.0);
  return message;
}
}  // namespace

int main()
{
  using drone_swarm::ParseBatch;
  auto valid = ValidBatch();
  auto parsed = ParseBatch(valid, 5);
  Check(parsed.has_value(), "Jazzy header-only model identity accepted");
  Check(parsed->size() == 5 && !(*parsed)[0] && (*parsed)[1].has_value(),
        "subset activates exactly its named model");
  Check((*parsed)[1]->Pos().X() == 2.0 && (*parsed)[1]->Pos().Z() == 4.0,
        "reference coordinates preserved");
  Check(ParseBatch(gz::msgs::Pose_V(), 5).has_value(), "empty batch deactivates all");
  Check(!ParseBatch(valid, 0) && !ParseBatch(valid, 401), "capacity bounds enforced");
  Check(!ParseBatch(valid, 1), "model outside owned capacity rejected");
  auto duplicate = valid;
  *duplicate.add_pose() = valid.pose(0);
  Check(!ParseBatch(duplicate, 5), "duplicate identities reject whole batch");
  auto badFrame = valid;
  badFrame.mutable_pose(0)->mutable_header()->mutable_data(0)->set_value(0, "map");
  Check(!ParseBatch(badFrame, 5), "non-world coordinate frame rejected");
  auto noHeader = valid;
  noHeader.mutable_pose(0)->clear_header();
  noHeader.mutable_pose(0)->set_name("drone_002");
  Check(!ParseBatch(noHeader, 5), "Pose.name alone is not ROS transform identity");
  auto ambiguous = valid;
  *ambiguous.mutable_pose(0)->mutable_header()->add_data() = valid.pose(0).header().data(1);
  Check(!ParseBatch(ambiguous, 5), "ambiguous child header rejected");
  auto conflicting = valid;
  conflicting.mutable_pose(0)->set_name("ground");
  Check(!ParseBatch(conflicting, 5), "conflicting Gazebo and ROS identities rejected");
  for (const auto *name : {"ground", "drone_000", "drone_401", "drone_02",
                            "world::drone_002", "drone_002/visual", "drone_00x"})
    Check(!drone_swarm::DroneIndex(name, 400), "scene or malformed model name rejected");
  for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(), 501.0, -501.0})
  {
    auto invalid = valid;
    invalid.mutable_pose(0)->mutable_position()->set_x(value);
    Check(!ParseBatch(invalid, 5), "invalid x coordinate rejected");
  }
  auto underground = valid;
  underground.mutable_pose(0)->mutable_position()->set_z(0.49);
  Check(!ParseBatch(underground, 5), "below-ground reference rejected");
  auto zeroQuaternion = valid;
  zeroQuaternion.mutable_pose(0)->mutable_orientation()->set_w(0.0);
  Check(!ParseBatch(zeroQuaternion, 5), "zero quaternion rejected");
  auto nonfiniteQuaternion = valid;
  nonfiniteQuaternion.mutable_pose(0)->mutable_orientation()->set_x(
      std::numeric_limits<double>::quiet_NaN());
  Check(!ParseBatch(nonfiniteQuaternion, 5), "nonfinite quaternion rejected");
  auto scaledQuaternion = valid;
  scaledQuaternion.mutable_pose(0)->mutable_orientation()->set_w(2.0);
  Check(ParseBatch(scaledQuaternion, 5)->at(1)->Rot().W() == 1.0,
        "valid quaternion normalized");
  std::cout << "Fleet protocol validation passed.\n";
  return EXIT_SUCCESS;
}

#include <ariac_test/ariac_test.hpp>

AriacTest::AriacTest()
    : Node("tester"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
      ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
      planning_scene_()
{
  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);
  floor_robot_.setPlanningTime(10.0);
  floor_robot_.setNumPlanningAttempts(5);
  floor_robot_.allowReplanning(true);
  floor_robot_.setReplanAttempts(5);

  ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
  ceiling_robot_.setMaxVelocityScalingFactor(1.0);
  ceiling_robot_.setPlanningTime(10.0);
  ceiling_robot_.setNumPlanningAttempts(5);
  ceiling_robot_.allowReplanning(true);
  ceiling_robot_.setReplanAttempts(5);

  // Publishers
  test_status_pub_ = this->create_publisher<std_msgs::msg::String>("/test_status", 10);
  test_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(250.0)), std::bind(&AriacTest::test_timer_cb, this));

  // Subscribe to topics
  // test_sub_ = this->create_subscription<std_msgs::msg::String>(
  //     "test_status", 10, std::bind(&AriacTest::test_cb, this, std::placeholders::_1));

  rclcpp::SubscriptionOptions options;

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  options.callback_group = topic_cb_group_;

  orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 1,
                                                                  std::bind(&AriacTest::orders_cb, this, std::placeholders::_1), options);

  competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 1,
                                                                                        std::bind(&AriacTest::competition_state_cb, this, std::placeholders::_1), options);

  kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::kts1_camera_cb, this, std::placeholders::_1), options);

  kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::kts2_camera_cb, this, std::placeholders::_1), options);

  left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::left_bins_camera_cb, this, std::placeholders::_1), options);

  right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::right_bins_camera_cb, this, std::placeholders::_1), options);

  conveyor_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/conveyor_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::conveyor_camera_cb, this, std::placeholders::_1), options);

  breakbeam_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
      "/ariac/sensors/conveyor_breakbeam/change", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::breakbeam_cb, this, std::placeholders::_1), options);

  conveyor_parts_sub_ = this->create_subscription<ariac_msgs::msg::ConveyorParts>(
      "/ariac/conveyor_parts", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::conveyor_parts_cb, this, std::placeholders::_1), options);
  
  floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
      "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::floor_gripper_state_cb, this, std::placeholders::_1), options);

  ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
      "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::ceiling_gripper_state_cb, this, std::placeholders::_1), options);

  as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::as1_state_cb, this, std::placeholders::_1), options);

  as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::as2_state_cb, this, std::placeholders::_1), options);

  as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::as3_state_cb, this, std::placeholders::_1), options);

  as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&AriacTest::as4_state_cb, this, std::placeholders::_1), options);

  agv1_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv1_status", 10,
      std::bind(&AriacTest::agv1_status_cb, this, std::placeholders::_1), options);

  agv2_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv2_status", 10,
      std::bind(&AriacTest::agv2_status_cb, this, std::placeholders::_1), options);

  agv3_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv3_status", 10,
      std::bind(&AriacTest::agv3_status_cb, this, std::placeholders::_1), options);

  agv4_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv4_status", 10,
      std::bind(&AriacTest::agv4_status_cb, this, std::placeholders::_1), options);

  // Initialize service clients
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
  pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
  floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
  floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
  ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

AriacTest::~AriacTest()
{
  floor_robot_.~MoveGroupInterface();
  ceiling_robot_.~MoveGroupInterface();
}

void AriacTest::test_timer_cb()
{
  auto message = std_msgs::msg::String();
  message.data = test_status;
  test_status_pub_->publish(message);
}

// void test_cb(const std_msgs::msg::String & msg)
// {
//   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
// }

void AriacTest::orders_cb(
    const ariac_msgs::msg::Order::ConstSharedPtr msg)
{
  orders_.push_back(*msg);
}

void AriacTest::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
  competition_state_ = msg->competition_state;
}

void AriacTest::kts1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!kts1_camera_recieved_data)
  {
    RCLCPP_DEBUG(get_logger(), "Received data from kts1 camera");
    kts1_camera_recieved_data = true;
  }

  kts1_trays_ = msg->tray_poses;
  kts1_camera_pose_ = msg->sensor_pose;
}

void AriacTest::kts2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!kts2_camera_recieved_data)
  {
    RCLCPP_DEBUG(get_logger(), "Received data from kts2 camera");
    kts2_camera_recieved_data = true;
  }

  kts2_trays_ = msg->tray_poses;
  kts2_camera_pose_ = msg->sensor_pose;
}

void AriacTest::left_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!left_bins_camera_recieved_data)
  {
    RCLCPP_DEBUG(get_logger(), "Received data from left bins camera");
    left_bins_camera_recieved_data = true;
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

void AriacTest::right_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!right_bins_camera_recieved_data)
  {
    RCLCPP_DEBUG(get_logger(), "Received data from right bins camera");
    right_bins_camera_recieved_data = true;
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}

void AriacTest::conveyor_parts_cb(
    const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg
)
{
  if (!conveyor_parts_recieved_data)
  {
    RCLCPP_DEBUG(get_logger(), "Received data from conveyor parts");
    conveyor_parts_recieved_data = true;
  }

  conveyor_parts_expected_ = msg->parts;
}

void AriacTest::conveyor_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!conveyor_camera_recieved_data)
  {
    RCLCPP_DEBUG(get_logger(), "Received data from conveyor camera");
    conveyor_camera_recieved_data = true;
  }
  
  conveyor_part_detected_ = msg->part_poses;
  conveyor_camera_pose_ = msg->sensor_pose;
}

void AriacTest::breakbeam_cb(
    const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg)
{
    if (!breakbeam_received_data)
    {
      RCLCPP_DEBUG(get_logger(), "Received data from conveyor breakbeam");
      breakbeam_received_data = true;
      breakbeam_pose_ = FrameWorldPose(msg->header.frame_id);
    }

    breakbeam_status = msg->object_detected;
    ariac_msgs::msg::PartPose part_to_add;
    auto detection_time = now();
    float prev_distance = 0;
    int count = 0;

    if (breakbeam_status)
    {
      // Lock conveyor_parts_ mutex
      std::lock_guard<std::mutex> lock(conveyor_parts_mutex);

      for (auto part : conveyor_part_detected_)
      {
        geometry_msgs::msg::Pose part_pose = MultiplyPose(conveyor_camera_pose_, part.pose);
        float distance = abs(part_pose.position.y - breakbeam_pose_.position.y);
        if (count == 0)
        {
          part_to_add = part;
          detection_time = now();
          prev_distance = distance;
          count++;
        }
        else
        {
          if (distance < prev_distance)
          {
            part_to_add = part;
            detection_time = now();
            prev_distance = distance;
          }
        }
      }
      conveyor_parts_.emplace_back(part_to_add, detection_time);
    }
}

void AriacTest::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
  floor_gripper_state_ = *msg;
}

void AriacTest::ceiling_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
  ceiling_gripper_state_ = *msg;
}

void AriacTest::as1_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}

void AriacTest::as2_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}

void AriacTest::as3_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}

void AriacTest::as4_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

void AriacTest::agv1_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[1] = msg->location;
}

void AriacTest::agv2_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[2] = msg->location;
}

void AriacTest::agv3_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[3] = msg->location;
}

void AriacTest::agv4_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[4] = msg->location;
}

geometry_msgs::msg::Pose AriacTest::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
  KDL::Frame f1;
  KDL::Frame f2;

  tf2::fromMsg(p1, f1);
  tf2::fromMsg(p2, f2);

  KDL::Frame f3 = f1 * f2;

  return tf2::toMsg(f3);
}

void AriacTest::LogPose(geometry_msgs::msg::Pose p)
{
  tf2::Quaternion q(
      p.orientation.x,
      p.orientation.y,
      p.orientation.z,
      p.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  roll *= 180 / M_PI;
  pitch *= 180 / M_PI;
  yaw *= 180 / M_PI;

  RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
              p.position.x, p.position.y, p.position.z,
              roll, pitch, yaw);
}

geometry_msgs::msg::Pose AriacTest::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;

  return pose;
}

geometry_msgs::msg::Pose AriacTest::FrameWorldPose(std::string frame_id)
{
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try
  {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

double AriacTest::GetYaw(geometry_msgs::msg::Pose pose)
{
  tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

geometry_msgs::msg::Quaternion AriacTest::QuaternionFromRPY(double r, double p, double y)
{
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;

  q.setRPY(r, p, y);

  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  return q_msg;
}

moveit_msgs::msg::CollisionObject AriacTest::CreateCollisionObject(
  std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("ariac_test");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  return collision;
}

void AriacTest::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
  planning_scene_.applyCollisionObject(CreateCollisionObject(name, mesh_file, model_pose));
}

void AriacTest::AddModelsToPlanningScene()
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {
      {"bin1", std::pair<double, double>(-1.9, 3.375)},
      {"bin2", std::pair<double, double>(-1.9, 2.625)},
      {"bin3", std::pair<double, double>(-2.65, 2.625)},
      {"bin4", std::pair<double, double>(-2.65, 3.375)},
      {"bin5", std::pair<double, double>(-1.9, -3.375)},
      {"bin6", std::pair<double, double>(-1.9, -2.625)},
      {"bin7", std::pair<double, double>(-2.65, -2.625)},
      {"bin8", std::pair<double, double>(-2.65, -3.375)}};

  geometry_msgs::msg::Pose bin_pose;
  for (auto const &bin : bin_positions)
  {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    objects.push_back(CreateCollisionObject(bin.first, "bin.stl", bin_pose));
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions = {
      {"as1", std::pair<double, double>(-7.3, 3)},
      {"as2", std::pair<double, double>(-12.3, 3)},
      {"as3", std::pair<double, double>(-7.3, -3)},
      {"as4", std::pair<double, double>(-12.3, -3)},
  };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const &station : assembly_station_positions)
  {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

    objects.push_back(CreateCollisionObject(station.first, "assembly_station.stl", assembly_station_pose));
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
      {"as1_insert", std::pair<double, double>(-7.7, 3)},
      {"as2_insert", std::pair<double, double>(-12.7, 3)},
      {"as3_insert", std::pair<double, double>(-7.7, -3)},
      {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;

  for (auto const &insert : assembly_insert_positions)
  {
    // assembly_insert_pose.position.x = insert.second.first;
    // assembly_insert_pose.position.y = insert.second.second;
    // assembly_insert_pose.position.z = 1.011;
    // assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

    std::string frame_name = insert.first + "_frame";

    objects.push_back(CreateCollisionObject(insert.first, "assembly_insert.stl", FrameWorldPose(frame_name)));
  }

  geometry_msgs::msg::Pose conveyor_pose;
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

  objects.push_back(CreateCollisionObject("conveyor", "conveyor.stl", conveyor_pose));

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

  objects.push_back(CreateCollisionObject("kts1_table", "kit_tray_table.stl", kts1_table_pose));

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

  objects.push_back(CreateCollisionObject("kts2_table", "kit_tray_table.stl", kts2_table_pose));

  if (!planning_scene_.applyCollisionObjects(objects)) {
    RCLCPP_WARN(get_logger(), "Unable to add objects to planning scene");
  }
}

geometry_msgs::msg::Quaternion AriacTest::SetRobotOrientation(double rotation)
{
  tf2::Quaternion tf_q;
  tf_q.setRPY(0, 3.14159, rotation);

  geometry_msgs::msg::Quaternion q;

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q;
}

// Test Status Setter function
void AriacTest::SetTestStatus(std::string message)
{
  test_status = message;
}


bool AriacTest::FloorRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success)
  {
    return static_cast<bool>(floor_robot_.execute(plan));
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

bool AriacTest::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_.execute(trajectory));
}

std::pair<bool,moveit_msgs::msg::RobotTrajectory> AriacTest::FloorRobotPlanCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return std::make_pair(false, trajectory);
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return std::make_pair(true, trajectory);
}

void AriacTest::FloorRobotWaitForAttach(double timeout)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached)
  {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    FloorRobotMoveCartesian(waypoints, 0.01, 0.01, true);

    usleep(500);

    if (now() - start > rclcpp::Duration::from_seconds(timeout))
    {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

void AriacTest::FloorRobotSendHome()
{
  // Move floor robot to home joint state
  RCLCPP_INFO_STREAM(get_logger(), "Moving Floor Robot to home position");
  floor_robot_.setNamedTarget("home");
  FloorRobotMovetoTarget();
}

bool AriacTest::FloorRobotSetGripperState(bool enable)
{
  if (floor_gripper_state_.enabled == enable)
  {
    if (floor_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else
      RCLCPP_INFO(get_logger(), "Already disabled");

    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto future = floor_robot_gripper_enable_->async_send_request(request);
  future.wait();

  if (!future.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

bool AriacTest::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{
  // Move gripper into tool changer
  RCLCPP_INFO_STREAM(get_logger(), "Changing gripper to " << gripper_type << " gripper");

  auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z, SetRobotOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true))
    return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

  if (gripper_type == "trays")
  {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  }
  else if (gripper_type == "parts")
  {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto future = floor_robot_tool_changer_->async_send_request(request);
  future.wait();
  if (!future.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true))
    return false;

  return true;
}

bool AriacTest::FloorRobotPickandPlaceTray(int tray_id, int agv_num)
{
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick up kit tray " << tray_id << " and place on AGV " << agv_num);

  // Check if kit tray is on one of the two tables
  geometry_msgs::msg::Pose tray_pose;
  std::string station;
  bool found_tray = false;

  // Check table 1
  for (auto tray : kts1_trays_)
  {
    if (tray.id == tray_id)
    {
      station = "kts1";
      tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
      found_tray = true;
      break;
    }
  }
  // Check table 2
  if (!found_tray)
  {
    for (auto tray : kts2_trays_)
    {
      if (tray.id == tray_id)
      {
        station = "kts2";
        tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
        found_tray = true;
        break;
      }
    }
  }
  if (!found_tray)
    return false;

  double tray_rotation = GetYaw(tray_pose);

  RCLCPP_INFO_STREAM(get_logger(), "Found kit tray " << tray_id << " on ktting tray station " << station << " moving to pick location");

  // Move floor robot to the corresponding kit tray table
  if (station == "kts1")
  {
    floor_robot_.setJointValueTarget(floor_kts1_js_);
  }
  else
  {
    floor_robot_.setJointValueTarget(floor_kts2_js_);
  }
  FloorRobotMovetoTarget();

  // Change gripper to tray gripper
  if (floor_gripper_state_.type != "tray_gripper")
  {
    FloorRobotChangeGripper(station, "trays");
  }

  // Move to tray
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  FloorRobotSetGripperState(true);

  FloorRobotWaitForAttach(3.0);

  RCLCPP_INFO_STREAM(get_logger(), "Picked kit tray " << tray_id);

  // Add kit tray to planning scene
  std::string tray_name = "kit_tray_" + std::to_string(tray_id);
  AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
  floor_robot_.attachObject(tray_name);

  order_planning_scene_objects_.push_back(tray_name);

  // Move up slightly
  waypoints.clear();

  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                              tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.05, 0.05, true);

  if (station == "kts1")
  {
    floor_robot_.setJointValueTarget(floor_kts1_js_);
  }
  else
  {
    floor_robot_.setJointValueTarget(floor_kts2_js_);
  }
  FloorRobotMovetoTarget();

  RCLCPP_INFO_STREAM(get_logger(), "Moving tray to AGV " << agv_num);

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

  FloorRobotMovetoTarget();

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
  auto agv_rotation = GetYaw(agv_tray_pose);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true);

  FloorRobotSetGripperState(false);

  floor_robot_.detachObject(tray_name);

  LockAGVTray(agv_num);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1, false);

  return true;
}

bool AriacTest::FloorRobotTrayTest()
{

  RCLCPP_INFO_STREAM(get_logger(), "Testing Floor Robot Pick for Trays");
  geometry_msgs::msg::Pose tray_pose;
  std::string station;
  std::vector<int> agv_nums{1, 2, 3, 4};
  std::vector<ariac_msgs::msg::KitTrayPose> trays_kts1;
  std::vector<ariac_msgs::msg::KitTrayPose> trays_kts2;

  trays_kts1.insert(trays_kts1.begin(), kts1_trays_.begin(), kts1_trays_.end());
  trays_kts2.insert(trays_kts2.begin(), kts2_trays_.begin(), kts2_trays_.end());

  // Change Gripper
  if (floor_gripper_state_.type != "tray_gripper")
  {
    std::string station = "kts1";
    floor_robot_.setJointValueTarget(floor_kts1_js_);
    FloorRobotMovetoTarget();
    FloorRobotChangeGripper(station, "trays");
  }

  for (auto tray : trays_kts1)
  { 

    floor_robot_.setJointValueTarget(floor_kts1_js_);
    FloorRobotMovetoTarget();
    station = "kts1";
    tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
    double tray_rotation = GetYaw(tray_pose);

    // Move to tray
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);

    FloorRobotSetGripperState(true);

    FloorRobotWaitForAttach(3.0);

    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray.id);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
    floor_robot_.attachObject(tray_name);

    order_planning_scene_objects_.push_back(tray_name);

    // Move up slightly
    waypoints.clear();

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.05, 0.05, true);

    floor_robot_.setJointValueTarget(floor_kts1_js_);
    
    FloorRobotMovetoTarget();

    sleep(1);

    // Cause Failure
    // FloorRobotSetGripperState(false);
    // floor_robot_.detachObject(tray_name); 
    // sleep(1);

    // Check if tray is attached to robot
    if (!floor_gripper_state_.attached)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Unsuccessfull in picking tray" << tray_name << " from kts1");

      std::string message = "Failed to pick " + tray_name + " from kts1";
      SetTestStatus(message);

      return true;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Moving tray to AGV " << agv_nums.back());

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_nums.back())]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

    FloorRobotMovetoTarget();

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_nums.back()) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true);

    FloorRobotSetGripperState(false);

    floor_robot_.detachObject(tray_name);

    LockAGVTray(agv_nums.back());

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1, false);

    RCLCPP_INFO_STREAM(get_logger(), "Done placing on AGV  " << agv_nums.back());
    agv_nums.pop_back();
    }

    FloorRobotSendHome();

    for (auto tray : trays_kts2)
  { 

    floor_robot_.setJointValueTarget(floor_kts2_js_);
    FloorRobotMovetoTarget();

    station = "kts2";
    tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
    double tray_rotation = GetYaw(tray_pose);

    // Move to tray
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);

    FloorRobotSetGripperState(true);

    FloorRobotWaitForAttach(3.0);

    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray.id);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
    floor_robot_.attachObject(tray_name);

    order_planning_scene_objects_.push_back(tray_name);

    // Move up slightly
    waypoints.clear();

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.05, 0.05, true);

    floor_robot_.setJointValueTarget(floor_kts2_js_);
    
    FloorRobotMovetoTarget();

    // Check if tray is attached to robot
    sleep(1);

    if (!floor_gripper_state_.attached)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Unsuccessfull in picking tray" << tray_name << " from kts2");

      std::string message = "Failed to pick " + tray_name + " from kts2";
      SetTestStatus(message);

      return true;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Moving tray to AGV " << agv_nums.back());

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_nums.back())]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

    FloorRobotMovetoTarget();

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_nums.back()) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true);

    FloorRobotSetGripperState(false);

    floor_robot_.detachObject(tray_name);

    LockAGVTray(agv_nums.back());

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1, false);

    RCLCPP_INFO_STREAM(get_logger(), "Done placing on AGV  " << agv_nums.back());
    agv_nums.pop_back();

    }

    FloorRobotSendHome();
    RCLCPP_INFO_STREAM(get_logger(), "Finished Testing Floor Robot Pick for Trays");

    SetTestStatus("Passed!");

    return true;
  }

bool AriacTest::FloorRobotBinTest()
{
  RCLCPP_INFO_STREAM(get_logger(), "Testing Floor Robot Pick from Bins");

  //Change gripper at station 1
  if (floor_gripper_state_.type != "part_gripper")
  {
    std::string station = "kts1";
    floor_robot_.setJointValueTarget(floor_kts1_js_);
    FloorRobotMovetoTarget();
    FloorRobotChangeGripper(station, "parts");
  }

  // Create two new vectors to hold initially detected parts
  std::vector<ariac_msgs::msg::PartPose> right_parts;
  std::vector<ariac_msgs::msg::PartPose> left_parts;
  right_parts.insert(right_parts.begin(), right_bins_parts_.begin(), right_bins_parts_.end());
  left_parts.insert(left_parts.begin(), left_bins_parts_.begin(), left_bins_parts_.end());

  RCLCPP_INFO_STREAM(get_logger(), "Parts in right bin " << right_bins_parts_.size());


  for (auto part : right_parts)
  { 
    geometry_msgs::msg::Pose part_pose;
    ariac_msgs::msg::Part part_to_pick;

    part_to_pick = part.part;
    part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
    double part_rotation = GetYaw(part_pose);

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["right_bins"]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);
    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(3.0);

    // Add part to planning scene
    std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
    AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_ = part_to_pick;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true);

    // Uncomment the following lines to cause failure
    // FloorRobotSetGripperState(false);
    // floor_robot_.detachObject(part_name);

    sleep(2);

    if (!floor_gripper_state_.attached)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Unsuccessfull in picking the " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the  right bin");
      
      // floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
      // FloorRobotMovetoTarget();

      std::string message = "Failed to pick " + part_colors_[part_to_pick.color] + " " + part_types_[part_to_pick.type] + " in right bins";
      SetTestStatus(message);
      
      return true;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Successfully picked the " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the  right bin");

    // Move up over right bins 
    // floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["right_bins"]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Move to disposal bin
    floor_robot_.setJointValueTarget(floor_disposal_poses_[1]);
    FloorRobotMovetoTarget();

    // Drop part in disposable bin
    FloorRobotSetGripperState(false);
    floor_robot_.detachObject(part_name);
    planning_scene_.removeCollisionObjects({part_name});     
    waypoints.clear();
    sleep(1);

  }

  FloorRobotSendHome();

  RCLCPP_INFO_STREAM(get_logger(), "Parts to be picked in left bin " << left_bins_parts_.size());

  for (auto part : left_parts)
  { 
    geometry_msgs::msg::Pose part_pose;
    ariac_msgs::msg::Part part_to_pick;

    part_to_pick = part.part;
    part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
    double part_rotation = GetYaw(part_pose);

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["left_bins"]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();



      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                    part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

      waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                    part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

      FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);
      FloorRobotSetGripperState(true);
      FloorRobotWaitForAttach(3.0);

      // Add part to planning scene
      std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
      AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
      floor_robot_.attachObject(part_name);
      floor_robot_attached_part_ = part_to_pick;

      // Move up slightly
      waypoints.clear();
      waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                    part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));
      FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true);

      sleep(1);

      if (!floor_gripper_state_.attached)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Unsuccessfull in picking the " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the  right bin");

      std::string message = "Failed to pick " + part_colors_[part_to_pick.color] + " " + part_types_[part_to_pick.type] + " in left bins";
      SetTestStatus(message);

      return true;
    }

      RCLCPP_INFO_STREAM(get_logger(), "Successfully picked the " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the left bin");

      // Move up over right bins 
      // floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["right_bins"]);
      floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
      FloorRobotMovetoTarget();

      // Move to disposal bin
      floor_robot_.setJointValueTarget(floor_disposal_poses_[4]);
      FloorRobotMovetoTarget();

      // Drop part in disposable bin
      FloorRobotSetGripperState(false);
      floor_robot_.detachObject(part_name);
      planning_scene_.removeCollisionObjects({part_name});     
      waypoints.clear();
      sleep(1);

  }

  SetTestStatus("Passed!");

  FloorRobotSendHome();
  return true;
}

bool AriacTest::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
{
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the bins");

  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose;
  bool found_part = false;
  std::string bin_side;

  // Check left bins
  for (auto part : left_bins_parts_)
  {
    if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
    {
      part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
      found_part = true;
      bin_side = "left_bins";
      break;
    }
  }
  // Check right bins
  if (!found_part)
  {
    for (auto part : right_bins_parts_)
    {
      if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
      {
        part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "right_bins";
        break;
      }
    }
  }
  if (!found_part)
  {
    RCLCPP_INFO(get_logger(), "Unable to locate part in the bins");
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "Found part in " << bin_side << " checking gripper type and moving to pick location");

  double part_rotation = GetYaw(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper")
  {
    std::string station;
    if (part_pose.position.y < 0)
    {
      station = "kts1";
    }
    else
    {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    FloorRobotChangeGripper(station, "parts");
  }

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  FloorRobotSetGripperState(true);

  FloorRobotWaitForAttach(3.0);

  RCLCPP_INFO_STREAM(get_logger(), "Picked Up the " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the bins");

  // Add part to planning scene
  std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;

  order_planning_scene_objects_.push_back(part_name);

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.2, SetRobotOrientation(part_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true);

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  return true;
}

bool AriacTest::FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick)
{
  if (conveyor_parts_expected_.empty())
  {
    RCLCPP_INFO(get_logger(), "No parts expected on the conveyor");
    return false;
  }
  for (auto parts : conveyor_parts_expected_){
    if (parts.part.type == part_to_pick.type && parts.part.color == part_to_pick.color){
      RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the conveyor");
      break;
    }
    else if (parts == conveyor_parts_expected_.back()){
      RCLCPP_INFO(get_logger(), "Unable to locate part on the conveyor");
      return false;
    }
  }

  bool found_part = false;
  bool part_picked = false;
  int num_tries = 0;
  geometry_msgs::msg::Pose part_pose;
  rclcpp::Duration elapsed_time(0, 0);
  builtin_interfaces::msg::Duration time_to_pick;
  rclcpp::Time detection_time;

  // Change gripper at Kitting Tray Station 2
  if (floor_gripper_state_.type != "part_gripper")
  {
    floor_robot_.setJointValueTarget(floor_kts2_js_);
    FloorRobotMovetoTarget();
    FloorRobotChangeGripper("kts2", "parts");
  }
  RCLCPP_INFO_STREAM(get_logger(), "Moving Floor Robot to Conveyor pick location");
  while(!part_picked && num_tries < 3){
    // Move robot to predefined pick location
    floor_robot_.setJointValueTarget(floor_conveyor_js_);
    FloorRobotMovetoTarget();

    // Find the requested part on the conveyor
    do {
        { 
        // Lock conveyor_parts_ mutex
        std::lock_guard<std::mutex> lock(conveyor_parts_mutex);
        auto it = conveyor_parts_.begin();
        for (; it != conveyor_parts_.end(); ) {
            auto part = it->first.part;
            if (part.type == part_to_pick.type && part.color == part_to_pick.color)
            {
              part_pose = MultiplyPose(conveyor_camera_pose_, it->first.pose);
              detection_time = it->second;

              elapsed_time = rclcpp::Time(now()) - detection_time;
              auto current_part_position_ = part_pose.position.y - (elapsed_time.seconds() * conveyor_speed_);
              // Check if part hasn't passed the pick location
              if (current_part_position_ > 0)
              {
                time_to_pick.sec = current_part_position_ / conveyor_speed_;
                // Check if part has more than 5 seconds to arrive at pick location
                if (time_to_pick.sec > 5.0)
                {
                  found_part = true;
                  conveyor_parts_.erase(it);
                  break;
                }
              }
              it = conveyor_parts_.erase(it);
            }
            else{
              ++it;
            }
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20000, "Waiting for %s %s to arrive on the conveyor", part_colors_[part_to_pick.color].c_str(), part_types_[part_to_pick.type].c_str());
        } // End lock_guard scope
      } while (!found_part);

    // Correct robot position to account for part offset on conveyor
    double part_rotation = GetYaw(part_pose);
    geometry_msgs::msg::Pose robot_pose = floor_robot_.getCurrentPose().pose;
    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, robot_pose.position.y,
                                  part_pose.position.z + 0.15, SetRobotOrientation(part_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.5, 0.5, true);

    auto elapsed_time_ = rclcpp::Time(now()) - detection_time;
    auto current_part_position_ = part_pose.position.y - (elapsed_time.seconds() * conveyor_speed_);
    // Check if part hasn't passed the pick location
    if (current_part_position_ < 0)
    {
      RCLCPP_INFO(get_logger(), "Part has passed the pick location");
      found_part = false;
      num_tries++;
      continue;
    }

    // Plan trajectory to pickup part
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x,  robot_pose.position.y,
                                  part_pose.position.z + part_heights_[part_to_pick.type],SetRobotOrientation(part_rotation)));

    auto trajectory = FloorRobotPlanCartesian(waypoints, 0.5, 0.5, true);
    if (!trajectory.first)
    {
      found_part = false;
      num_tries++;
      continue;
    }

    // Wait for part to arrive at pick location
    auto trajectory_time = trajectory.second.joint_trajectory.points.back().time_from_start;
    while (rclcpp::Time(now()).nanoseconds() < (detection_time + elapsed_time + time_to_pick - trajectory_time).nanoseconds() - 2.75e8)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20000, "Waiting for part to arrive at pick location");
    }

    // Execute trajectory to pickup part
    FloorRobotSetGripperState(true);
    floor_robot_.execute(trajectory.second);

    auto start_time = rclcpp::Time(now());

    while(!floor_gripper_state_.attached){
      if((rclcpp::Time(now())-start_time ).seconds() > 2.0)
        break;
    }



    if(floor_gripper_state_.attached)
    {
      part_pose.position.y = robot_pose.position.y;
      // Add part to planning scene
      std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
      AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
      RCLCPP_INFO_STREAM(get_logger(), "Attached " << part_name << " to robot");
      floor_robot_.attachObject(part_name);
      order_planning_scene_objects_.push_back(part_name);
      floor_robot_attached_part_ = part_to_pick;
      part_picked = true;
    }

    waypoints.clear();
    waypoints.push_back(BuildPose(robot_pose.position.x, robot_pose.position.y,
                                    robot_pose.position.z + 0.2, SetRobotOrientation(part_rotation)));
    
    FloorRobotMoveCartesian(waypoints, 1, 1, false);

    // Move up
    // waypoints.clear();
    // waypoints.push_back(BuildPose(robot_pose.position.x, robot_pose.position.y,
    //                               robot_pose.position.z + 0.3, SetRobotOrientation(0)));
    // FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);
    
    num_tries++;
  }

  return true;
}

bool AriacTest::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
  if (!floor_gripper_state_.attached)
  {
    RCLCPP_ERROR(get_logger(), "No part attached");
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "Placing the " << part_colors_[floor_robot_attached_part_.color] << " " << part_types_[floor_robot_attached_part_.type] << " on AGV " << agv_num << " in quadrant " << quadrant);

  // Move to agv
  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                    geometry_msgs::msg::Quaternion());

  auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + kit_tray_thickness_ + drop_height_,
                                SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.2, true);

  // Drop part in quadrant
  FloorRobotSetGripperState(false);

  std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                          "_" + part_types_[floor_robot_attached_part_.type];
  floor_robot_.detachObject(part_name);

  waypoints.clear();
  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + 0.2,
                                SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1, false);

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  return true;
}

void AriacTest::CeilingRobotSendHome()
{
  // Move ceiling robot to home joint state
  RCLCPP_INFO_STREAM(get_logger(), "Moving ceiling robot to home position");
  ceiling_robot_.setNamedTarget("home");
  CeilingRobotMovetoTarget();
}

bool AriacTest::CeilingRobotSetGripperState(bool enable)
{
  if (ceiling_gripper_state_.enabled == enable)
  {
    if (ceiling_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else
      RCLCPP_INFO(get_logger(), "Already disabled");

    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto future = ceiling_robot_gripper_enable_->async_send_request(request);
  future.wait();

  if (!future.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

void AriacTest::CeilingRobotWaitForAttach(double timeout)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  while (!ceiling_gripper_state_.attached)
  {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(500);

    if (now() - start > rclcpp::Duration::from_seconds(timeout))
    {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

bool AriacTest::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Wait for part to be attached
  rclcpp::Time start = now();

  auto rotation = GetYaw(FrameWorldPose("as"+std::to_string(station)+"_insert_frame"));

  bool assembled = false;

  double R[3][3] = {cos(rotation), -sin(rotation), 0,
                      sin(rotation), cos(rotation), 0,
                      0, 0, 1};

  double dx = R[0][0] * part.install_direction.x + R[0][1] * part.install_direction.y + R[0][2] * part.install_direction.z;
  double dy = R[1][0] * part.install_direction.x + R[1][1] * part.install_direction.y + R[1][2] * part.install_direction.z;
  double dz = R[2][0] * part.install_direction.x + R[2][1] * part.install_direction.y + R[2][2] * part.install_direction.z;

  geometry_msgs::msg::Pose pose = ceiling_robot_.getCurrentPose().pose;

  pose.position.x += dx * (assembly_offset_ + 0.005);
  pose.position.y += dy * (assembly_offset_ + 0.005);
  pose.position.z += dz * (assembly_offset_ + 0.005);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose);

  auto ret = CeilingRobotPlanCartesian(waypoints, 0.01, 0.01, false);

  if (ret.first) {
    ceiling_robot_.asyncExecute(ret.second);
  } else {
    return false;
  }

  while (!assembled)
  {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

    // Check if part is assembled
    switch (part.part.type)
    {
    case ariac_msgs::msg::Part::BATTERY:
      assembled = assembly_station_states_[station].battery_attached;
      break;
    case ariac_msgs::msg::Part::PUMP:
      assembled = assembly_station_states_[station].pump_attached;
      break;
    case ariac_msgs::msg::Part::SENSOR:
      assembled = assembly_station_states_[station].sensor_attached;
      break;
    case ariac_msgs::msg::Part::REGULATOR:
      assembled = assembly_station_states_[station].regulator_attached;
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid part type");
      return false;
    }
  }

  ceiling_robot_.stop();

  RCLCPP_INFO(get_logger(), "Part is assembled");

  return true;
}

bool AriacTest::CeilingRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success)
  {
    return static_cast<bool>(ceiling_robot_.execute(plan));
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

bool AriacTest::CeilingRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
  rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(ceiling_robot_.execute(trajectory));
}

std::pair<bool,moveit_msgs::msg::RobotTrajectory> AriacTest::CeilingRobotPlanCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return std::make_pair(false, trajectory);
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
  rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return std::make_pair(true, trajectory);
}

bool AriacTest::CeilingRobotMoveToAssemblyStation(int station)
{
  RCLCPP_INFO_STREAM(get_logger(), "Moving ceiling robot to assembly station " << station);
  switch (station)
  {
  case 1:
    ceiling_robot_.setJointValueTarget(ceiling_as1_js_);
    break;
  case 2:
    ceiling_robot_.setJointValueTarget(ceiling_as2_js_);
    break;
  case 3:
    ceiling_robot_.setJointValueTarget(ceiling_as3_js_);
    break;
  case 4:
    ceiling_robot_.setJointValueTarget(ceiling_as4_js_);
    break;
  default:
    RCLCPP_WARN(get_logger(), "Not a valid assembly station");
    return false;
  }

  return CeilingRobotMovetoTarget();
}

bool AriacTest::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
  RCLCPP_INFO_STREAM(get_logger(), "Determining waypoints to pick " << part_types_[part.part.type]);
  double part_rotation = GetYaw(part.pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;

  double dx = 0;
  double dy = 0;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY)
  {
    dx = battery_grip_offset_ * cos(part_rotation);
    dy = battery_grip_offset_ * sin(part_rotation);
  }

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                part.pose.position.z + part_heights_[part.part.type] + pick_offset_, SetRobotOrientation(part_rotation)));

  RCLCPP_INFO_STREAM(get_logger(), "Moving ceiling robot to pick " << part_types_[part.part.type]);
  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, false);

  CeilingRobotSetGripperState(true);

  CeilingRobotWaitForAttach(5.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
  AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
  ceiling_robot_.attachObject(part_name);
  ceiling_robot_attached_part_ = part.part;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(part.pose.position.x, part.pose.position.y,
                                part.pose.position.z + 0.3,
                                SetRobotOrientation(part_rotation)));

  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  return true;
}

bool AriacTest::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Check that part is attached and matches part to assemble
  if (!ceiling_gripper_state_.attached)
  {
    RCLCPP_WARN(get_logger(), "No part attached");
    return false;
  }

  if (part.part != ceiling_robot_attached_part_)
  {
    RCLCPP_WARN(get_logger(), "Incorrect part attached for this assembly");
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(),"Attempting to assemble a " << part_colors_[part.part.color] << " " << part_types_[part.part.type]);
  // Calculate assembled pose in world frame
  std::string insert_frame_name;
  switch (station)
  {
  case 1:
    insert_frame_name = "as1_insert_frame";
    break;
  case 2:
    insert_frame_name = "as2_insert_frame";
    break;
  case 3:
    insert_frame_name = "as3_insert_frame";
    break;
  case 4:
    insert_frame_name = "as4_insert_frame";
    break;
  default:
    RCLCPP_WARN(get_logger(), "Not a valid assembly station");
    return false;
  }

  // Calculate robot positions at assembly and approach
  KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

  KDL::Frame insert;
  tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

  KDL::Frame part_assemble;
  tf2::fromMsg(part.assembled_pose.pose, part_assemble);

  KDL::Frame part_to_gripper;

  // Build approach waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (part.part.type == ariac_msgs::msg::Part::BATTERY)
  {
    tf2::fromMsg(BuildPose(battery_grip_offset_, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);
    KDL::Vector up(0, 0, 0.1);
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
  }
  else
  {
    tf2::fromMsg(BuildPose(0, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
  }

  // Move to approach position
  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  // Move to just before assembly position
  waypoints.clear();
  waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -assembly_offset_) * part_assemble * part_to_gripper));
  CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, true);

  CeilingRobotWaitForAssemble(station, part);

  CeilingRobotSetGripperState(false);

  std::string part_name = part_colors_[ceiling_robot_attached_part_.color] +
                          "_" + part_types_[ceiling_robot_attached_part_.type];
  ceiling_robot_.detachObject(part_name);

  waypoints.clear();
  KDL::Vector away;
  if (part.part.type == ariac_msgs::msg::Part::REGULATOR) {
    away = KDL::Vector(-0.1, 0, 0);
  } else {
    away = KDL::Vector(0, 0, 0.1);
  }

  waypoints.push_back(tf2::toMsg(insert * KDL::Frame(away) * part_assemble * part_to_gripper));

  CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, false);

  return true;
}

bool AriacTest::CompleteOrders()
{
  // Wait for first order to be published
  while (orders_.size() == 0)
  {
  }

  bool success;
  while (true)
  {
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
    {
      success = false;
      break;
    }

    if (orders_.size() == 0)
    {
      if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
      {
        // wait for more orders
        RCLCPP_INFO(get_logger(), "Waiting for orders...");
        while (orders_.size() == 0)
        {
        }
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Completed all orders");
        success = true;
        break;
      }
    }

    current_order_ = orders_.front();
    orders_.erase(orders_.begin());
    // int kitting_agv_num = -1;

    if (current_order_.type == ariac_msgs::msg::Order::KITTING)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Current order is: " << current_order_.id << " of type KITTING");
      AriacTest::CompleteKittingTask(current_order_.kitting_task);
    }
    else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Current order is: " << current_order_.id << " of type ASSEMBLY");
      AriacTest::CompleteAssemblyTask(current_order_.assembly_task);
    }
    else if (current_order_.type == ariac_msgs::msg::Order::COMBINED)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Current order is: " << current_order_.id << " of type COMBINED");
      AriacTest::CompleteCombinedTask(current_order_.combined_task);
    }


    AriacTest::SubmitOrder(current_order_.id);
  }
  return success;
}


bool AriacTest::CompleteKittingTask(ariac_msgs::msg::KittingTask task)
{
  FloorRobotSendHome();

  if(agv_locations_[task.agv_number] != ariac_msgs::msg::AGVStatus::KITTING)
  {
    MoveAGV(task.agv_number,ariac_msgs::srv::MoveAGV::Request::KITTING);
  }

  FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

  bool found;
  for (auto kit_part : task.parts)
  {
    found = FloorRobotPickBinPart(kit_part.part);
    if (!found)
    {
      FloorRobotPickConveyorPart(kit_part.part);
    }
    FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
  }

  // Check quality
  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = current_order_.id;
  auto future = quality_checker_->async_send_request(request);
  future.wait();

  if (!future.get()->all_passed)
  {
    RCLCPP_ERROR(get_logger(), "Issue with shipment");
  }

  //Remove objects from planning scene
  planning_scene_.removeCollisionObjects(order_planning_scene_objects_);

  order_planning_scene_objects_.clear();

  MoveAGV(task.agv_number, task.destination);

  return true;
}

bool AriacTest::AssemblyTest()
{

  // Wait for first order to be published
  while (orders_.size() == 0)
  {
  }

  current_order_ = orders_.front();
  // orders_.erase(orders_.begin());
  
  ariac_msgs::msg::AssemblyTask task = current_order_.assembly_task;

  // Send AGVs to assembly station
  for (auto const &agv : task.agv_numbers)
  {
    int destination;
    if (task.station == ariac_msgs::msg::AssemblyTask::AS1 || task.station == ariac_msgs::msg::AssemblyTask::AS3)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
    }
    else if (task.station == ariac_msgs::msg::AssemblyTask::AS2 || task.station == ariac_msgs::msg::AssemblyTask::AS4)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
    }

    LockAGVTray(agv);
    MoveAGV(agv, destination);
  }

  CeilingRobotMoveToAssemblyStation(task.station);

  // Get Assembly Poses
  RCLCPP_INFO_STREAM(get_logger(), "Getting pre assembly poses for order " << current_order_.id);
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_.id;
  auto future = pre_assembly_poses_getter_->async_send_request(request);

  RCLCPP_INFO(get_logger(), "Waiting for pre assembly poses");

  future.wait();

  RCLCPP_INFO(get_logger(), "Recieved pre assembly poses");

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses;

  auto result = future.get();
  
  if (result->valid_id)
  {
    RCLCPP_INFO(get_logger(), "Valid id");

    try {
      int len = result->parts.size();
      RCLCPP_INFO_STREAM(get_logger(), "There are " << std::to_string(len) << " parts");
    }
    catch (...) {
      RCLCPP_INFO(get_logger(), "Unable to access future result");
    }

    agv_part_poses = result->parts;

    if (agv_part_poses.size() == 0)
    {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
      return false;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
    return false;
  }

  for (auto const &part_to_assemble : task.parts)
  {
    // Check if matching part exists in agv_parts
    bool part_exists = false;
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part = part_to_assemble.part;
    for (auto const &agv_part : agv_part_poses)
    {
      if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color)
      {
        part_exists = true;
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    if (!part_exists)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << " and color: " << part_to_assemble.part.color << " not found on tray");
      continue;
    }

    // Pick up part
    CeilingRobotPickAGVPart(part_to_pick);

    CeilingRobotMoveToAssemblyStation(task.station);

    // Assemble Part to insert
    CeilingRobotAssemblePart(task.station, part_to_assemble);

    CeilingRobotMoveToAssemblyStation(task.station);

  }

  if (assembly_station_states_[task.station].battery_attached && assembly_station_states_[task.station].pump_attached && assembly_station_states_[task.station].regulator_attached && assembly_station_states_[task.station].sensor_attached)
    {
      SetTestStatus("Passed");
    }

    else
    {
      if(!assembly_station_states_[task.station].battery_attached){
        SetTestStatus("Failed Battery Assembly");
      }

      else if(!assembly_station_states_[task.station].pump_attached){
        SetTestStatus("Failed Pump Assembly");
      }

      else if(!assembly_station_states_[task.station].sensor_attached){
        SetTestStatus("Failed Sensor Assembly");
      }

      else if(!assembly_station_states_[task.station].regulator_attached){
        SetTestStatus("Failed Regualtor Assembly");
      }

      else SetTestStatus("Failed");
        
    }

  return true;
}

bool AriacTest::CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task)
{
  // Send AGVs to assembly station
  for (auto const &agv : task.agv_numbers)
  {
    int destination;
    if (task.station == ariac_msgs::msg::AssemblyTask::AS1 || task.station == ariac_msgs::msg::AssemblyTask::AS3)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
    }
    else if (task.station == ariac_msgs::msg::AssemblyTask::AS2 || task.station == ariac_msgs::msg::AssemblyTask::AS4)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
    }

    LockAGVTray(agv);
    MoveAGV(agv, destination);
  }

  CeilingRobotMoveToAssemblyStation(task.station);

  // Get Assembly Poses
  RCLCPP_INFO_STREAM(get_logger(), "Getting pre assembly poses for order " << current_order_.id);
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_.id;
  auto future = pre_assembly_poses_getter_->async_send_request(request);

  RCLCPP_INFO(get_logger(), "Waiting for pre assembly poses");

  future.wait();

  RCLCPP_INFO(get_logger(), "Recieved pre assembly poses");

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses;

  auto result = future.get();
  
  if (result->valid_id)
  {
    RCLCPP_INFO(get_logger(), "Valid id");

    try {
      int len = result->parts.size();
      RCLCPP_INFO_STREAM(get_logger(), "There are " << std::to_string(len) << " parts");
    }
    catch (...) {
      RCLCPP_INFO(get_logger(), "Unable to access future result");
    }

    agv_part_poses = result->parts;

    if (agv_part_poses.size() == 0)
    {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
      return false;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
    return false;
  }

  for (auto const &part_to_assemble : task.parts)
  {
    // Check if matching part exists in agv_parts
    bool part_exists = false;
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part = part_to_assemble.part;
    for (auto const &agv_part : agv_part_poses)
    {
      if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color)
      {
        part_exists = true;
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    if (!part_exists)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << " and color: " << part_to_assemble.part.color << " not found on tray");
      continue;
    }

    // Pick up part
    CeilingRobotPickAGVPart(part_to_pick);

    CeilingRobotMoveToAssemblyStation(task.station);

    // Assemble Part to insert
    CeilingRobotAssemblePart(task.station, part_to_assemble);

    CeilingRobotMoveToAssemblyStation(task.station);
  }

  return true;
}

bool AriacTest::CompleteCombinedTask(ariac_msgs::msg::CombinedTask task)
{
  RCLCPP_INFO_STREAM(get_logger(),"Starting Combined Task");
  // Decide on a tray to use
  int id;
  if (kts1_trays_.size() != 0)
  {
    id = kts1_trays_[0].id;
    RCLCPP_INFO_STREAM(get_logger(), "Using tray " << id << " from kts1");
  }
  else if (kts2_trays_.size() != 0)
  {
    id = kts2_trays_[0].id;
    RCLCPP_INFO_STREAM(get_logger(), "Using tray " << id << " from kts2");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "No trays available.");
    return false;
  }

  // Decide which AGV to use
  int agv_number;
  if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS2)
  {
    agv_number = 1;
    RCLCPP_INFO_STREAM(get_logger(),"Using AGV " << agv_number << " for assembly since station is AS1 or AS2");
  }
  else
  {
    agv_number = 4;
    RCLCPP_INFO_STREAM(get_logger(),"Using AGV " << agv_number << " for assembly since station is AS3 or AS4");
  }

  MoveAGV(agv_number, ariac_msgs::srv::MoveAGV::Request::KITTING);

  FloorRobotPickandPlaceTray(id, agv_number);

  int count = 1;
  bool found;
  for (auto assembly_part : task.parts)
  {
    found = FloorRobotPickBinPart(assembly_part.part);
    if (!found)
    {
      FloorRobotPickConveyorPart(assembly_part.part);
    }
    FloorRobotPlacePartOnKitTray(agv_number, count);
    count++;
  }

  int destination;
  if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS3)
  {
    destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
  }
  else
  {
    destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
  }

  //Remove objects from planning scene
  planning_scene_.removeCollisionObjects(order_planning_scene_objects_);

  order_planning_scene_objects_.clear();

  MoveAGV(agv_number, destination);

  CeilingRobotMoveToAssemblyStation(task.station);

  // Get Assembly Poses
  RCLCPP_INFO_STREAM(get_logger(), "Getting pre assembly poses for order " << current_order_.id);
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_.id;
  auto future = pre_assembly_poses_getter_->async_send_request(request);

  future.wait();

  auto result = future.get();

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses;
  if (result->valid_id)
  {
    agv_part_poses = result->parts;

    if (agv_part_poses.size() == 0)
    {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
      return false;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
    return false;
  }

  for (auto const &part_to_assemble : task.parts)
  {
    // Check if matching part exists in agv_parts
    bool part_exists = false;
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part = part_to_assemble.part;
    for (auto const &agv_part : agv_part_poses)
    {
      if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color)
      {
        part_exists = true;
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    if (!part_exists)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_types_[part_to_assemble.part.type] << " and color: " << part_colors_[part_to_assemble.part.color] << " not found on tray");
      continue;
    }

    // Pick up part
    CeilingRobotPickAGVPart(part_to_pick);

    CeilingRobotMoveToAssemblyStation(task.station);

    // Assemble Part to insert
    CeilingRobotAssemblePart(task.station, part_to_assemble);

    CeilingRobotMoveToAssemblyStation(task.station);
  }

  return true;
}

bool AriacTest::StartCompetition()
{
  // Wait for competition state to be ready
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY)
  {
  }

  RCLCPP_INFO_STREAM(get_logger(), "Starting competition");

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future = client->async_send_request(request);
  future.wait();

  return future.get()->success;
}

bool AriacTest::EndCompetition()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  RCLCPP_INFO(get_logger(), "Ending competition.");

  auto future = client->async_send_request(request);
  future.wait();

  return future.get()->success;
}

bool AriacTest::SubmitOrder(std::string order_id)
{
  RCLCPP_INFO_STREAM(get_logger(), "Submitting order " << order_id);
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  auto future = client->async_send_request(request);
  future.wait();

  return future.get()->success;
}

bool AriacTest::LockAGVTray(int agv_num)
{
  RCLCPP_INFO_STREAM(get_logger(), "Locking Tray to AGV" << agv_num);

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future = client->async_send_request(request);
  future.wait();

  return future.get()->success;
}

bool AriacTest::UnlockAGVTray(int agv_num)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_unlock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future = client->async_send_request(request);
  future.wait();

  return future.get()->success;
}

bool AriacTest::MoveAGV(int agv_num, int destination)
{
  RCLCPP_INFO_STREAM(get_logger(), "Moving AGV" << agv_num << " to " << agv_destination_[destination]);
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

  client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;

  auto future = client->async_send_request(request);

  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(20);

  rclcpp::Time start = get_clock()->now();

  while (get_clock()->now() - start < timeout){
    if (agv_locations_[agv_num] == destination) {
      return true;
    }
  }

  RCLCPP_INFO_STREAM(get_logger(), "Unable to move AGV" << agv_num << " to " << agv_destination_[destination]);
  return false;
}
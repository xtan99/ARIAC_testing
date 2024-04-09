#include <ariac_test/ariac_test.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto tester = std::make_shared<AriacTest>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(tester);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Start Competition
  tester->StartCompetition();

  tester->AddModelsToPlanningScene();

  sleep(2);

  // Move Robots to Home Poses
  tester->FloorRobotSendHome();
  tester->CeilingRobotSendHome();

  // Complete Orders
  tester->FloorRobotTrayTest();

  sleep(5);
  
  tester->EndCompetition();

  rclcpp::shutdown();
}
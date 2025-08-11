// 这个就是能够改动acm，忽略指定mesh之间碰撞的程序，可以通过同名的launch文件启动它。

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <unordered_map>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("acm_add_collision_pairs");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("acm_add_collision_pairs_node");

  auto get_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  auto apply_client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

  get_client->wait_for_service();
  apply_client->wait_for_service();

  auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  get_req->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto get_res = get_client->async_send_request(get_req);
  if (rclcpp::spin_until_future_complete(node, get_res) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to get planning scene");
    return 1;
  }

  moveit_msgs::msg::AllowedCollisionMatrix acm = get_res.get()->scene.allowed_collision_matrix;

  // 构建 name -> index 映射
  std::unordered_map<std::string, size_t> name_to_idx;
  for (size_t i = 0; i < acm.entry_names.size(); ++i)
    name_to_idx[acm.entry_names[i]] = i;

  auto get_or_create_index = [&](const std::string& name) {
    if (name_to_idx.count(name)) return name_to_idx[name];
    size_t idx = acm.entry_names.size();
    acm.entry_names.push_back(name);
    for (auto& row : acm.entry_values)
      row.enabled.push_back(false);
    moveit_msgs::msg::AllowedCollisionEntry new_entry;
    new_entry.enabled.resize(acm.entry_names.size(), false);
    acm.entry_values.push_back(new_entry);
    name_to_idx[name] = idx;
    return idx;
  };

  // 要添加的碰撞对
  std::vector<std::pair<std::string, std::string>> allow_pairs = {
    {"base_link_inertia", "frame_mesh"},
    {"frame_mesh", "scanner_mesh"},
    {"ft300_mounting_plate", "realsense_camera"},
    // {"ft300_sensor", "realsense_camera"},
    {"ft300_mounting_plate", "ft300_sensor"},
    {"ft300_sensor", "robotiq_85_base_link"},
    {"realsense_camera", "wrist_3_link"},
  };

  for (const auto& pair : allow_pairs)
  {
    size_t i = get_or_create_index(pair.first);
    size_t j = get_or_create_index(pair.second);
    acm.entry_values[i].enabled[j] = true;
    acm.entry_values[j].enabled[i] = true;
    RCLCPP_INFO(LOGGER, "Allowed collision: %s <-> %s", pair.first.c_str(), pair.second.c_str());
  }

  auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_req->scene.is_diff = true;
  apply_req->scene.allowed_collision_matrix = acm;

  auto apply_res = apply_client->async_send_request(apply_req);
  if (rclcpp::spin_until_future_complete(node, apply_res) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to apply planning scene");
    return 1;
  }

  if (apply_res.get()->success)
    RCLCPP_INFO(LOGGER, "ACM updated successfully");
  else
    RCLCPP_WARN(LOGGER, "ApplyPlanningScene call failed");

  rclcpp::shutdown();
  return 0;
}

// Test version
// #include <rclcpp/rclcpp.hpp>
// #include <moveit_msgs/srv/get_planning_scene.hpp>
// #include <moveit_msgs/srv/apply_planning_scene.hpp>
// #include <moveit_msgs/msg/allowed_collision_entry.hpp>
// #include <unordered_map>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("modify_acm_service_merge");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("modify_acm_service_merge_node");

//   auto get_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
//   auto apply_client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

//   // 等待服务
//   RCLCPP_INFO(LOGGER, "Waiting for services...");
//   get_client->wait_for_service();
//   apply_client->wait_for_service();

//   // 请求当前 scene
//   auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
//   get_req->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

//   auto get_res = get_client->async_send_request(get_req);
//   if (rclcpp::spin_until_future_complete(node, get_res) != rclcpp::FutureReturnCode::SUCCESS) {
//     RCLCPP_ERROR(LOGGER, "Failed to call get_planning_scene");
//     return 1;
//   }

//   moveit_msgs::msg::AllowedCollisionMatrix acm = get_res.get()->scene.allowed_collision_matrix;

//   // 构建名称->index 映射
//   std::unordered_map<std::string, size_t> name_to_idx;
//   for (size_t i = 0; i < acm.entry_names.size(); ++i)
//     name_to_idx[acm.entry_names[i]] = i;

//   // 要加入的新 entry 名称
//   std::string name_a = "Box_0", name_b = "panda_link0";

//   auto get_or_create_index = [&](const std::string& name) {
//     if (name_to_idx.find(name) != name_to_idx.end()) return name_to_idx[name];
//     size_t new_idx = acm.entry_names.size();
//     acm.entry_names.push_back(name);
//     moveit_msgs::msg::AllowedCollisionEntry new_entry;
//     new_entry.enabled.resize(acm.entry_names.size(), false);
//     // 为之前已有的 entry 增加一列
//     for (auto& entry : acm.entry_values)
//       entry.enabled.push_back(false);
//     acm.entry_values.push_back(new_entry);
//     name_to_idx[name] = new_idx;
//     return new_idx;
//   };

//   size_t idx_a = get_or_create_index(name_a);
//   size_t idx_b = get_or_create_index(name_b);

//   // 设置互相允许碰撞
//   acm.entry_values[idx_a].enabled[idx_b] = true;
//   acm.entry_values[idx_b].enabled[idx_a] = true;

//   // 构建 apply 请求
//   auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
//   apply_req->scene.is_diff = true;
//   apply_req->scene.allowed_collision_matrix = acm;

//   auto apply_res = apply_client->async_send_request(apply_req);
//   if (rclcpp::spin_until_future_complete(node, apply_res) != rclcpp::FutureReturnCode::SUCCESS) {
//     RCLCPP_ERROR(LOGGER, "Failed to apply planning scene");
//     return 1;
//   }

//   if (apply_res.get()->success)
//     RCLCPP_INFO(LOGGER, "Successfully updated ACM with Box_0 <-> Box_1");
//   else
//     RCLCPP_WARN(LOGGER, "ApplyPlanningScene call failed");

//   rclcpp::shutdown();
//   return 0;
// }

// Replace version
// #include <rclcpp/rclcpp.hpp>
// #include <moveit_msgs/srv/apply_planning_scene.hpp>
// #include <moveit_msgs/msg/allowed_collision_entry.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("modify_acm_service");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("modify_acm_service_node");

//   // 创建 service client
//   auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

//   // 等待服务可用
//   RCLCPP_INFO(LOGGER, "Waiting for /apply_planning_scene service...");
//   if (!client->wait_for_service(std::chrono::seconds(5))) {
//     RCLCPP_ERROR(LOGGER, "Service not available.");
//     return 1;
//   }

//   // 构造请求
//   auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
//   auto& scene = request->scene;
//   scene.is_diff = true;

//   auto& acm = scene.allowed_collision_matrix;
//   acm.entry_names = {"Box_0", "Box_1"};

//   moveit_msgs::msg::AllowedCollisionEntry entry0;
//   entry0.enabled = {true, true};  // Box_0 与 Box_0, Box_1 允许碰撞

//   moveit_msgs::msg::AllowedCollisionEntry entry1;
//   entry1.enabled = {true, true};  // Box_1 与 Box_0, Box_1 允许碰撞

//   acm.entry_values = {entry0, entry1};

//   // 异步调用服务
//   RCLCPP_INFO(LOGGER, "Sending ACM update request to /apply_planning_scene...");
//   auto result_future = client->async_send_request(request);

//   // 等待并处理响应
//   if (rclcpp::spin_until_future_complete(node, result_future) ==
//       rclcpp::FutureReturnCode::SUCCESS)
//   {
//     if (result_future.get()->success)
//       RCLCPP_INFO(LOGGER, "ACM successfully updated.");
//     else
//       RCLCPP_WARN(LOGGER, "Service call returned, but ACM update failed.");
//   }
//   else
//   {
//     RCLCPP_ERROR(LOGGER, "Service call failed.");
//   }

//   rclcpp::shutdown();
//   return 0;
// }



// Topic version
// #include <rclcpp/rclcpp.hpp>
// #include <moveit_msgs/msg/planning_scene.hpp>
// #include <moveit_msgs/msg/allowed_collision_entry.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("modify_acm_topic");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("modify_acm_topic_node");

//   // 创建 Publisher
//   auto planning_scene_pub =
//       node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

//   // 等待 MoveGroup 节点订阅者连接
//   RCLCPP_INFO(LOGGER, "Waiting for subscribers to planning_scene topic...");
//   while (planning_scene_pub->get_subscription_count() < 1) {
//     rclcpp::sleep_for(std::chrono::milliseconds(200));
//   }
//   RCLCPP_INFO(LOGGER, "Subscriber connected. Publishing ACM update...");

//   // 设置要修改的 ACM
//   moveit_msgs::msg::PlanningScene scene_msg;
//   scene_msg.is_diff = true;

//   // ACM 是一个对称矩阵
//   auto& acm = scene_msg.allowed_collision_matrix;
//   acm.entry_names = {"Box_0", "Box_1"};

//   moveit_msgs::msg::AllowedCollisionEntry entry0;
//   entry0.enabled = {true, true};  // Box_0 与 Box_0, Box_1 允许碰撞

//   moveit_msgs::msg::AllowedCollisionEntry entry1;
//   entry1.enabled = {true, true};  // Box_1 与 Box_0, Box_1 允许碰撞

//   acm.entry_values = {entry0, entry1};

//   // 发布 ACM 更新
//   planning_scene_pub->publish(scene_msg);
//   RCLCPP_INFO(LOGGER, "Published ACM diff: ignoring collision between 'Box_0' and 'Box_1'");

//   // 稍作等待，确保消息送达
//   rclcpp::sleep_for(std::chrono::seconds(1));
//   rclcpp::shutdown();
//   return 0;
// }

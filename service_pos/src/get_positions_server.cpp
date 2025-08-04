#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "service_pos/srv/get_positions.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PositionServiceNode : public rclcpp::Node
{
public:
  PositionServiceNode()
  : Node("position_service")
  {
    // Inisialisasi service
    service_ = this->create_service<service_pos::srv::GetPositions>(
      "get_positions",
      std::bind(&PositionServiceNode::handle_service, this, _1, _2)
    );

    // Subscription untuk posisi robot
    robot_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/robot_pos", 10,
      std::bind(&PositionServiceNode::robot_callback, this, _1)
    );

    // Subscription untuk posisi target
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_pos", 10,
      std::bind(&PositionServiceNode::target_callback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Service aktif di: /get_positions");
  }

private:
  // Callback untuk menyimpan posisi robot terakhir
  void robot_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    last_robot_pos_ = *msg;
  }

  // Callback untuk menyimpan posisi target terakhir
  void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    last_target_pos_ = *msg;
  }

  // Callback handler untuk service request
  void handle_service(
    const std::shared_ptr<service_pos::srv::GetPositions::Request> request,
    std::shared_ptr<service_pos::srv::GetPositions::Response> response)
  {
    (void)request;  // request tidak digunakan

    response->robot_pos = last_robot_pos_;
    response->target_pos = last_target_pos_;

    RCLCPP_INFO(this->get_logger(), "\n--- Posisi Dikirim---\n"
                                    "Robot  : [x = %.2f, y = %.2f, z = %.2f]\n"
                                    "Target : [x = %.2f, y = %.2f, z = %.2f]\n",
      response->robot_pos.x, response->robot_pos.y, response->robot_pos.z,
      response->target_pos.x, response->target_pos.y, response->target_pos.z);
  }

  geometry_msgs::msg::Point last_robot_pos_;
  geometry_msgs::msg::Point last_target_pos_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Service<service_pos::srv::GetPositions>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionServiceNode>());
  rclcpp::shutdown();
  return 0;
}


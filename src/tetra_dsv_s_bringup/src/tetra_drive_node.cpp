// TETRA-DSV-S drive-board node: /cmd_vel in, wheel odometry + joint states out.
//
// This is the piece scout_bringup got from CAN + ugv_sdk. The TETRA has no CAN;
// it is RS-232 all the way down, so nothing of that stack carries over except
// the shape of the launch file.
//
// Deliberately does NOT publish odom -> base_footprint. robot_localization's
// EKF owns that transform, exactly as in simulation, so that swapping between
// sim and hardware changes no TF topology. Publishing it here as well would
// give TF two parents for the same frame and make it non-deterministic.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tetra_dsv_s_bringup/drive_board.hpp"

using namespace std::chrono_literals;

namespace
{

// ======================================================================
//  조정 파라미터 기본값 — 전부 ROS 파라미터로 덮어쓸 수 있습니다
// ======================================================================

// 시리얼 포트. udev/99-tetra-serial.rules 가 심볼릭 링크를 만들어 줍니다.
// 원시 장치명(/dev/ttyUSB0)은 부팅 순서에 따라 뒤바뀌므로 쓰지 말 것.
constexpr const char * DEFAULT_PORT = "/dev/tetra_drive";

// 제어 주기. 벤더 ROS 1 노드가 30 Hz 였고(tetraDS.cpp:513) 115200 bps 에서
// BV 왕복 1회가 약 1.6 ms 이므로 여유가 있습니다.
constexpr double DEFAULT_RATE_HZ = 30.0;

// ⚠ 휠 트랙 — HANDOFF §2-(1) 을 읽고 손댈 것.
// 매뉴얼 Para5 는 0.438, 벤더 드라이버는 0.377, 벤더 URDF 는 0.378 입니다.
// 차체 폭이 0.430 이라 0.438 이면 바퀴가 밖으로 튀어나옵니다. 속도 제어에서
// 펌웨어는 트랙을 쓰지 않고 좌우 바퀴 속도를 mm/s 로 그대로 받으므로, 여기
// 들어갈 값은 "실제 물리 트랙"입니다. 실기에서 줄자로 재기 전까지는 벤더와
// 같은 값으로 둡니다. 시뮬(0.438)과 일부러 다릅니다.
constexpr double DEFAULT_WHEEL_SEPARATION = 0.377;
constexpr double DEFAULT_WHEEL_RADIUS = 0.1015;  // Para4 203 mm, 벤더와 일치

// 펌웨어 Para8 이 1500 mm/s 이므로 그 이상은 어차피 잘립니다.
constexpr double DEFAULT_MAX_LINEAR = 1.0;    // m/s
constexpr double DEFAULT_MAX_ANGULAR = 1.5;   // rad/s

// cmd_vel 이 이 시간 동안 없으면 정지시킵니다. Nav2 가 죽거나 WiFi 가 끊겼을 때
// 마지막 속도로 계속 달리는 것을 막는 유일한 장치입니다.
constexpr double DEFAULT_CMD_TIMEOUT = 0.5;   // s

}  // namespace

namespace tetra_dsv_s_bringup
{

class TetraDriveNode : public rclcpp::Node
{
public:
  TetraDriveNode()
  : Node("tetra_drive")
  {
    port_ = declare_parameter("port", std::string(DEFAULT_PORT));
    rate_hz_ = declare_parameter("rate_hz", DEFAULT_RATE_HZ);
    wheel_separation_ = declare_parameter("wheel_separation", DEFAULT_WHEEL_SEPARATION);
    wheel_radius_ = declare_parameter("wheel_radius", DEFAULT_WHEEL_RADIUS);
    max_linear_ = declare_parameter("max_linear_velocity", DEFAULT_MAX_LINEAR);
    max_angular_ = declare_parameter("max_angular_velocity", DEFAULT_MAX_ANGULAR);
    cmd_timeout_ = declare_parameter("cmd_vel_timeout", DEFAULT_CMD_TIMEOUT);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));

    if (!board_.open(port_)) {
      RCLCPP_FATAL(get_logger(), "cannot open %s: %s", port_.c_str(),
        board_.last_error().c_str());
      throw std::runtime_error("drive board open failed");
    }

    // The board boots in position mode and ignores BV until told otherwise.
    // This is a silent failure: velocities are accepted, nothing moves.
    if (!board_.set_velocity_mode()) {
      RCLCPP_ERROR(get_logger(), "set_velocity_mode failed: %s",
        board_.last_error().c_str());
    }
    if (!board_.set_servo(true)) {
      RCLCPP_ERROR(get_logger(), "servo on failed: %s", board_.last_error().c_str());
    }
    board_.reset_odometry();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odometry", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    bumper_pub_ = create_publisher<std_msgs::msg::UInt8>("bumper", 10);
    emg_pub_ = create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
      });

    last_cmd_time_ = now();
    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {tick();});

    RCLCPP_INFO(get_logger(), "tetra_drive up on %s (track %.4f m, r %.4f m)",
      port_.c_str(), wheel_separation_, wheel_radius_);
  }

private:
  void tick()
  {
    const auto stamp = now();

    // Watchdog. A stale command is treated as "stop", never as "keep going".
    double vx = 0.0, wz = 0.0;
    if ((stamp - last_cmd_time_).seconds() <= cmd_timeout_) {
      vx = std::clamp(last_cmd_.linear.x, -max_linear_, max_linear_);
      wz = std::clamp(last_cmd_.angular.z, -max_angular_, max_angular_);
    } else if (!timed_out_) {
      RCLCPP_WARN(get_logger(), "cmd_vel timeout, holding stop");
    }
    timed_out_ = (stamp - last_cmd_time_).seconds() > cmd_timeout_;

    // Differential drive. Same form the vendor uses, just written directly in
    // m/s instead of routing through RPM and back.
    const double half_track = wheel_separation_ / 2.0;
    const int left_mm = static_cast<int>(std::lround((vx - wz * half_track) * 1000.0));
    const int right_mm = static_cast<int>(std::lround((vx + wz * half_track) * 1000.0));

    DriveState st;
    if (!board_.set_velocity(left_mm, right_mm, st)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "BV round trip failed: %s", board_.last_error().c_str());
      return;
    }

    if (st.emergency && !emergency_) {
      RCLCPP_ERROR(get_logger(), "EMERGENCY STOP engaged at the robot");
    }
    emergency_ = st.emergency;

    std_msgs::msg::Bool emg_msg;
    emg_msg.data = st.emergency;
    emg_pub_->publish(emg_msg);

    std_msgs::msg::UInt8 bump_msg;
    bump_msg.data = st.bumper;
    bumper_pub_->publish(bump_msg);

    publish_odometry(stamp, st, vx, wz);
    publish_joints(stamp, vx, wz);
  }

  void publish_odometry(
    const rclcpp::Time & stamp, const DriveState & st, double vx, double wz)
  {
    // Firmware integrates the pose for us; using it rather than re-integrating
    // here keeps this node stateless and avoids a second source of drift.
    // theta_raw is 0.1 deg units per the vendor's odometry scaling.
    const double x = st.x_mm / 1000.0;
    const double y = st.y_mm / 1000.0;
    const double yaw = static_cast<double>(st.theta_raw) * 0.1 * M_PI / 180.0;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    msg.twist.twist.linear.x = vx;
    msg.twist.twist.angular.z = wz;

    // The EKF reads only vx and wz from this message (see ekf.yaml odom0_config)
    // so the pose covariance is advisory; the twist figures are the ones that
    // matter and they reflect encoder slip, not truth.
    msg.pose.covariance[0] = 0.05;
    msg.pose.covariance[7] = 0.05;
    msg.pose.covariance[35] = 0.10;
    msg.twist.covariance[0] = 0.01;
    msg.twist.covariance[35] = 0.05;
    odom_pub_->publish(msg);
  }

  void publish_joints(const rclcpp::Time & stamp, double vx, double wz)
  {
    const double half_track = wheel_separation_ / 2.0;
    const double dt = (stamp - last_tick_).seconds();
    last_tick_ = stamp;
    if (dt > 0.0 && dt < 1.0) {
      left_pos_ += (vx - wz * half_track) / wheel_radius_ * dt;
      right_pos_ += (vx + wz * half_track) / wheel_radius_ * dt;
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {left_pos_, right_pos_};
    js.velocity = {
      (vx - wz * half_track) / wheel_radius_,
      (vx + wz * half_track) / wheel_radius_};
    joint_pub_->publish(js);
  }

  DriveBoard board_;
  std::string port_, odom_frame_, base_frame_;
  double rate_hz_, wheel_separation_, wheel_radius_;
  double max_linear_, max_angular_, cmd_timeout_;

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_, last_tick_;
  bool timed_out_ = false, emergency_ = false;
  double left_pos_ = 0.0, right_pos_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr bumper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emg_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tetra_dsv_s_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tetra_dsv_s_bringup::TetraDriveNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("tetra_drive"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

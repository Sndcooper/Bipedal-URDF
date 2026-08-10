// ===========================================================================
//  balance_point.cpp
//
//  Three-layer cascade balance controller for the simulated wheeled biped.
//  This is a deliberate line-for-line port of the STM32 firmware at
//    RelevantPhysicalRobotFiles/RC_mcu_IK_wireless/firmware/src/main.cpp
//  so that gains tuned here transfer directly to the hardware.
//
//  ---------------------------------------------------------------------------
//  WHY IT IS STRUCTURED THIS WAY
//  ---------------------------------------------------------------------------
//  Every deviation from the firmware is a place where a gain could behave
//  differently in simulation than on the robot, which would defeat the point
//  of the exercise. So the following firmware quirks are reproduced exactly,
//  even where a "cleaner" implementation exists:
//
//    * All angles are in DEGREES, not radians.
//    * Wheel velocity and position are in ENCODER COUNTS, not SI units.
//      kp_vel / ki_vel / kp_pos / max_vel_cmd were all tuned in counts.
//    * The inner-loop integrator is UNSCALED (integral += error * dt, with
//      ki applied at the end), which is why ki = 670 is a sane value here and
//      would be absurd in a conventionally-scaled PID.
//    * The inner integrator has NO clamp. Only the safety latch and the
//      integral-kill switch reset it. This is a real property of the firmware
//      and a real source of windup behaviour worth studying.
//    * derivative = -gyro_rate  (derivative on measurement, not on error).
//    * Output is a PWM-equivalent clamped to +-255, converted to torque only
//      at the very last step.
//
//  ---------------------------------------------------------------------------
//  ATTITUDE: WHY THE IMU QUATERNION IS IGNORED
//  ---------------------------------------------------------------------------
//  gz-sensors reports IMU orientation relative to the sensor's INITIAL pose.
//  This robot spawns lying on its back, so the quaternion would read roughly
//  zero pitch while the robot is flat on the ground. Running the firmware's
//  own complementary filter on raw accelerometer and gyro data is both the
//  faithful choice and the only correct one.
//
//  ---------------------------------------------------------------------------
//  SIGN CONVENTION
//  ---------------------------------------------------------------------------
//  The URDF wheel axes are pre-mirrored (left 0 0 -1, right 0 0 1), so a
//  positive joint value drives both wheels forward. The firmware's
//    left = -output + turn_bias
//    right = -output - turn_bias
//  therefore ports across unchanged. `effort_sign` is the escape hatch if the
//  convention turns out inverted; see the verification note in the README.
// ===========================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr double kRadToDeg = 57.295779513082320876;
constexpr double kTwoPi = 6.283185307179586;

// Firmware cascade state machine (see main.cpp CascadeState).
enum class CascadeState : int
{
  Driving = 0,
  Rampdown = 1,
  Holding = 2
};

template <typename T>
T clampValue(T value, T low, T high)
{
  return std::max(low, std::min(value, high));
}
}  // namespace

class BalancePoint : public rclcpp::Node
{
public:
  BalancePoint()
  : rclcpp::Node("balance_point")
  {
    declareParameters();
    loadParameters();

    // --- Publishers ------------------------------------------------------
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_effort_controller/commands", rclcpp::QoS(10));

    // Full-rate telemetry for logging and plotting. Layout is documented in
    // publishTelemetry() and in the README.
    telemetry_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/balance/telemetry", rclcpp::QoS(50));

    // Latched so standing_mode still catches the arm state if it starts late.
    rclcpp::QoS armed_qos(1);
    armed_qos.transient_local();
    armed_pub_ = create_publisher<std_msgs::msg::Bool>("/balance/armed", armed_qos);

    state_pub_ = create_publisher<std_msgs::msg::String>(
      "/balance/state", rclcpp::QoS(10));

    // --- Subscriptions ---------------------------------------------------
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&BalancePoint::onImu, this, std::placeholders::_1));

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&BalancePoint::onJointState, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/balance/cmd_vel", rclcpp::QoS(10),
      std::bind(&BalancePoint::onCmdVel, this, std::placeholders::_1));

    // --- Services --------------------------------------------------------
    // Firmware "M" command / RC channel 7.
    arm_srv_ = create_service<std_srvs::srv::SetBool>(
      "/motors_enabled",
      std::bind(&BalancePoint::onArmRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    // Firmware "C" command: capture the current accel pitch as the zero.
    calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
      "/calibrate",
      std::bind(&BalancePoint::onCalibrate, this,
                std::placeholders::_1, std::placeholders::_2));

    // Firmware "R" command. Essential for clean A/B gain comparisons: without
    // it, a gain change leaves the old integrator loaded and the resulting
    // transient is easily mistaken for instability caused by the new gains.
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/reset_state",
      std::bind(&BalancePoint::onResetState, this,
                std::placeholders::_1, std::placeholders::_2));

    // Live parameter updates, so the GUI bridge and `ros2 param set` both work.
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&BalancePoint::onParameterUpdate, this, std::placeholders::_1));

    // --- Control timer ---------------------------------------------------
    // Node clock, not wall clock: with use_sim_time the loop must follow
    // simulation time or the effective dt drifts whenever RTF != 1.0.
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BalancePoint::controlStep, this));

    publishArmed();

    RCLCPP_INFO(get_logger(),
                "balance_point ready | %.0f Hz | Kp=%.1f Ki=%.1f Kd=%.2f | "
                "stall=%.3f N.m | motor_model=%s | DISARMED",
                control_rate_, kp_, ki_, kd_, stall_torque_,
                use_motor_model_ ? "on" : "off");
    RCLCPP_INFO(get_logger(),
                "Arm with: ros2 service call /motors_enabled "
                "std_srvs/srv/SetBool \"{data: true}\"");
  }

private:
  // =========================================================================
  //  Parameters
  // =========================================================================
  void declareParameters()
  {
    declare_parameter<double>("control_rate", 100.0);

    declare_parameter<double>("kp", 95.0);
    declare_parameter<double>("ki", 670.0);
    declare_parameter<double>("kd", 2.0);
    declare_parameter<double>("alpha", 0.96);
    declare_parameter<double>("base_target_angle", 0.0);
    declare_parameter<double>("max_safe_tilt", 25.0);

    declare_parameter<double>("kp_vel", 0.02);
    declare_parameter<double>("ki_vel", 0.001);
    declare_parameter<double>("tilt_bias_limit", 5.0);

    declare_parameter<double>("kp_pos", 0.5);
    declare_parameter<double>("max_vel_cmd", 800.0);
    declare_parameter<double>("hold_vel_threshold", 20.0);
    declare_parameter<double>("vel_alpha", 0.85);

    declare_parameter<double>("counts_per_rev", 1320.0);
    declare_parameter<double>("wheel_radius", 0.033);
    declare_parameter<double>("pwm_max", 255.0);
    declare_parameter<double>("stall_torque", 0.15);
    declare_parameter<double>("no_load_speed", 31.4);
    declare_parameter<bool>("use_motor_model", true);
    declare_parameter<double>("effort_sign", -1.0);

    declare_parameter<double>("telemetry_rate", 100.0);
    declare_parameter<double>("cmd_vel_timeout", 0.5);

    declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
    declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");
  }

  void loadParameters()
  {
    control_rate_ = get_parameter("control_rate").as_double();

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    alpha_ = get_parameter("alpha").as_double();
    base_target_angle_ = get_parameter("base_target_angle").as_double();
    max_safe_tilt_ = get_parameter("max_safe_tilt").as_double();

    kp_vel_ = get_parameter("kp_vel").as_double();
    ki_vel_ = get_parameter("ki_vel").as_double();
    tilt_bias_limit_ = get_parameter("tilt_bias_limit").as_double();

    kp_pos_ = get_parameter("kp_pos").as_double();
    max_vel_cmd_ = get_parameter("max_vel_cmd").as_double();
    hold_vel_threshold_ = get_parameter("hold_vel_threshold").as_double();
    vel_alpha_ = get_parameter("vel_alpha").as_double();

    counts_per_rev_ = get_parameter("counts_per_rev").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    pwm_max_ = get_parameter("pwm_max").as_double();
    stall_torque_ = get_parameter("stall_torque").as_double();
    no_load_speed_ = get_parameter("no_load_speed").as_double();
    use_motor_model_ = get_parameter("use_motor_model").as_bool();
    effort_sign_ = get_parameter("effort_sign").as_double();

    telemetry_rate_ = get_parameter("telemetry_rate").as_double();
    cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();

    left_wheel_name_ = get_parameter("left_wheel_joint").as_string();
    right_wheel_name_ = get_parameter("right_wheel_joint").as_string();
  }

  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      const std::string & n = p.get_name();

      // control_rate cannot change after the timer is created; rejecting it
      // explicitly is better than silently ignoring it, because a user who
      // thinks they changed the rate would misread every subsequent result.
      if (n == "control_rate") {
        result.successful = false;
        result.reason =
          "control_rate is fixed at startup; restart the node to change it";
        return result;
      }

      if (n == "kp") { kp_ = p.as_double(); }
      else if (n == "ki") { ki_ = p.as_double(); }
      else if (n == "kd") { kd_ = p.as_double(); }
      else if (n == "alpha") {
        const double a = p.as_double();
        if (a < 0.0 || a > 0.999) {
          result.successful = false;
          result.reason = "alpha must be within [0.0, 0.999]";
          return result;
        }
        alpha_ = a;
      }
      else if (n == "base_target_angle") { base_target_angle_ = p.as_double(); }
      else if (n == "max_safe_tilt") { max_safe_tilt_ = p.as_double(); }
      else if (n == "kp_vel") { kp_vel_ = p.as_double(); }
      else if (n == "ki_vel") { ki_vel_ = p.as_double(); }
      else if (n == "tilt_bias_limit") { tilt_bias_limit_ = p.as_double(); }
      else if (n == "kp_pos") { kp_pos_ = p.as_double(); }
      else if (n == "max_vel_cmd") { max_vel_cmd_ = p.as_double(); }
      else if (n == "hold_vel_threshold") { hold_vel_threshold_ = p.as_double(); }
      else if (n == "vel_alpha") {
        vel_alpha_ = clampValue(p.as_double(), 0.0, 0.99);
      }
      else if (n == "counts_per_rev") {
        const double c = p.as_double();
        if (c <= 0.0) {
          result.successful = false;
          result.reason = "counts_per_rev must be positive";
          return result;
        }
        counts_per_rev_ = c;
      }
      else if (n == "wheel_radius") { wheel_radius_ = p.as_double(); }
      else if (n == "pwm_max") { pwm_max_ = p.as_double(); }
      else if (n == "stall_torque") { stall_torque_ = p.as_double(); }
      else if (n == "no_load_speed") { no_load_speed_ = p.as_double(); }
      else if (n == "use_motor_model") { use_motor_model_ = p.as_bool(); }
      else if (n == "effort_sign") { effort_sign_ = p.as_double(); }
      else if (n == "telemetry_rate") { telemetry_rate_ = p.as_double(); }
      else if (n == "cmd_vel_timeout") { cmd_vel_timeout_ = p.as_double(); }
    }

    return result;
  }

  // =========================================================================
  //  Subscription callbacks
  // =========================================================================

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // imu_link is REP-103 aligned when the robot stands upright: the composite
    // of base_footprint->base_link and the imu_joint rotation is exactly the
    // identity (verified numerically), so imu_link axes are X forward, Y left,
    // Z up.
    //
    // An accelerometer measures specific force, so a body pitched nose-down by
    // theta about +Y reads a_body = R_y(theta)^T * (0, 0, +g), giving
    //   ax = +g sin(theta),  az = +g cos(theta)
    // and therefore  theta = atan2(ax, az).
    //
    // NOTE THE SIGN. The firmware uses atan2(-ax, ...) because its MPU6050 is
    // mounted with the opposite X convention. Carrying that negation over to
    // this REP-103-aligned frame would invert the feedback sign and turn the
    // balance loop into positive feedback - the robot would drive itself over
    // rather than catch itself. Positive pitch = leaning forward (nose down).
    const double ax = msg->linear_acceleration.x;
    const double az = msg->linear_acceleration.z;

    accel_pitch_raw_ = std::atan2(ax, az) * kRadToDeg;

    // Pitch rate about +Y. Sign matches GYRO_PITCH_SIGN = 1.0f in firmware.
    gyro_rate_ = msg->angular_velocity.y * kRadToDeg;

    imu_ready_ = true;
    last_imu_time_ = now();
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Resolve indices once; joint_states ordering is stable but not guaranteed.
    if (left_wheel_idx_ < 0 || right_wheel_idx_ < 0) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == left_wheel_name_) { left_wheel_idx_ = static_cast<int>(i); }
        if (msg->name[i] == right_wheel_name_) { right_wheel_idx_ = static_cast<int>(i); }
      }
      if (left_wheel_idx_ < 0 || right_wheel_idx_ < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Wheel joints '%s' / '%s' not present in /joint_states",
          left_wheel_name_.c_str(), right_wheel_name_.c_str());
        return;
      }
    }

    const auto li = static_cast<size_t>(left_wheel_idx_);
    const auto ri = static_cast<size_t>(right_wheel_idx_);

    if (msg->position.size() > std::max(li, ri)) {
      // Both axes are pre-mirrored in the URDF so positive = forward on each
      // side; no sign correction needed here (unlike the firmware, which has
      // to negate one encoder because the two motors are physically opposed).
      left_wheel_pos_rad_ = msg->position[li];
      right_wheel_pos_rad_ = msg->position[ri];
    }
    if (msg->velocity.size() > std::max(li, ri)) {
      left_wheel_vel_rad_ = msg->velocity[li];
      right_wheel_vel_rad_ = msg->velocity[ri];
    }

    joints_ready_ = true;
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // linear.x (m/s) -> encoder counts/sec, the unit the cascade works in.
    const double wheel_rad_per_s = msg->linear.x / wheel_radius_;
    cmd_velocity_counts_ = wheel_rad_per_s / kTwoPi * counts_per_rev_;

    // angular.z arrives already scaled to the +-425 PWM turn-bias range by
    // rc_teleop, carried on the z field to keep the message a plain Twist.
    cmd_turn_bias_ = msg->angular.z;

    last_cmd_time_ = now();
  }

  // =========================================================================
  //  Services
  // =========================================================================

  void onArmRequest(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data) {
      if (!imu_ready_ || !joints_ready_) {
        res->success = false;
        res->message = "Cannot arm: waiting for /imu and /joint_states";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
      resetState();
      motors_enabled_ = true;
      safety_latched_ = false;
      res->success = true;
      res->message = "Motors ARMED";
      RCLCPP_INFO(get_logger(), "Motors ARMED - integrators reset");
    } else {
      motors_enabled_ = false;
      safety_latched_ = false;  // disarm also clears the latch, as in firmware
      resetState();
      res->success = true;
      res->message = "Motors DISARMED";
      RCLCPP_INFO(get_logger(), "Motors DISARMED");
    }
    publishArmed();
  }

  void onCalibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!imu_ready_) {
      res->success = false;
      res->message = "No IMU data yet";
      return;
    }
    pitch_offset_ = accel_pitch_raw_;
    pitch_ = 0.0;
    resetState();

    res->success = true;
    res->message = "Calibrated: offset = " + std::to_string(pitch_offset_) + " deg";
    RCLCPP_INFO(get_logger(), "IMU calibrated, pitch offset = %.3f deg", pitch_offset_);
  }

  void onResetState(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    resetState();
    res->success = true;
    res->message = "Integrators and cascade state reset";
    RCLCPP_INFO(get_logger(), "State reset - ready for a clean run");
  }

  void resetState()
  {
    integral_ = 0.0;
    integral_vel_ = 0.0;
    tilt_bias_ = 0.0;
    vel_current_ = 0.0;
    target_velocity_ = 0.0;
    cascade_state_ = CascadeState::Holding;
    target_enc_pos_ = wheelPositionCounts();
    pos_error_ = 0.0;
    vel_error_ = 0.0;
    output_ = 0.0;
  }

  void publishArmed()
  {
    std_msgs::msg::Bool msg;
    msg.data = motors_enabled_;
    armed_pub_->publish(msg);
  }

  // =========================================================================
  //  Unit helpers
  // =========================================================================

  // Mean wheel position in encoder counts.
  double wheelPositionCounts() const
  {
    const double mean_rad = 0.5 * (left_wheel_pos_rad_ + right_wheel_pos_rad_);
    return mean_rad / kTwoPi * counts_per_rev_;
  }

  // Mean wheel velocity in encoder counts/sec.
  double wheelVelocityCounts() const
  {
    const double mean_rad_s = 0.5 * (left_wheel_vel_rad_ + right_wheel_vel_rad_);
    return mean_rad_s / kTwoPi * counts_per_rev_;
  }

  // PWM-equivalent command -> joint torque.
  //
  // With use_motor_model, available torque falls linearly with wheel speed and
  // reaches zero at no_load_speed, approximating back-EMF. Without it the
  // wheels behave as ideal torque sources, which is markedly stronger at speed
  // than the real JGA12V and makes derivative gains look better than they are.
  double pwmToTorque(double pwm, double wheel_vel_rad_s) const
  {
    const double duty = clampValue(pwm / pwm_max_, -1.0, 1.0);
    double torque = duty * stall_torque_;

    if (use_motor_model_ && no_load_speed_ > 1e-6) {
      const double speed_ratio =
        clampValue(wheel_vel_rad_s / no_load_speed_, -1.0, 1.0);
      torque -= speed_ratio * stall_torque_ * std::fabs(duty);
      torque = clampValue(torque, -stall_torque_, stall_torque_);
    }

    return torque;
  }

  // =========================================================================
  //  Control loop  (100 Hz)
  // =========================================================================

  void controlStep()
  {
    const rclcpp::Time current = now();

    if (!imu_ready_ || !joints_ready_) {
      publishWheelEffort(0.0, 0.0);
      return;
    }

    // --- dt --------------------------------------------------------------
    double dt = 1.0 / control_rate_;
    if (have_last_step_) {
      const double measured = (current - last_step_time_).seconds();
      // Guard against sim-time jumps (pause/resume, world reset).
      if (measured > 0.0 && measured < 0.5) {
        dt = measured;
      }
    }
    last_step_time_ = current;
    have_last_step_ = true;

    // --- Attitude: complementary filter (firmware main.cpp:390-394) -------
    const double accel_pitch = accel_pitch_raw_ - pitch_offset_;
    pitch_ = alpha_ * (pitch_ + gyro_rate_ * dt) + (1.0 - alpha_) * accel_pitch;

    // --- Wheel feedback ---------------------------------------------------
    const double vel_raw = wheelVelocityCounts();
    vel_current_ = vel_alpha_ * vel_current_ + (1.0 - vel_alpha_) * vel_raw;

    // --- Stale command guard ---------------------------------------------
    double cmd_velocity = cmd_velocity_counts_;
    double turn_bias = cmd_turn_bias_;
    if (cmd_vel_timeout_ > 0.0 && last_cmd_time_.nanoseconds() > 0) {
      if ((current - last_cmd_time_).seconds() > cmd_vel_timeout_) {
        cmd_velocity = 0.0;
        turn_bias = 0.0;
      }
    }

    // --- Safety latch (firmware main.cpp:766) -----------------------------
    if (std::fabs(pitch_) > max_safe_tilt_ && motors_enabled_) {
      motors_enabled_ = false;
      safety_latched_ = true;
      resetState();
      publishArmed();
      publishWheelEffort(0.0, 0.0);
      RCLCPP_WARN(get_logger(),
                  "SAFETY CUTOFF at %.1f deg (limit %.1f) - disarm/arm to clear",
                  pitch_, max_safe_tilt_);
      publishTelemetry(dt);
      return;
    }

    if (!motors_enabled_) {
      // Hold position target at the current wheel position so that arming
      // does not produce an immediate position-error step.
      target_enc_pos_ = wheelPositionCounts();
      publishWheelEffort(0.0, 0.0);
      publishTelemetry(dt);
      return;
    }

    // =====================================================================
    //  LAYER 3 (OUTER): position hold -> velocity target
    // =====================================================================
    const double enc_pos = wheelPositionCounts();
    const bool stick_active = std::fabs(cmd_velocity) > 1e-6;

    switch (cascade_state_) {
      case CascadeState::Driving:
        if (!stick_active) {
          cascade_state_ = CascadeState::Rampdown;
          target_velocity_ = 0.0;
          integral_vel_ = 0.0;  // flush so the integrator does not kick
        } else {
          target_velocity_ = clampValue(cmd_velocity, -max_vel_cmd_, max_vel_cmd_);
        }
        break;

      case CascadeState::Rampdown:
        if (stick_active) {
          cascade_state_ = CascadeState::Driving;
          integral_vel_ = 0.0;
        } else if (std::fabs(vel_current_) < hold_vel_threshold_) {
          target_enc_pos_ = enc_pos;   // latch where we stopped
          integral_vel_ = 0.0;
          cascade_state_ = CascadeState::Holding;
        } else {
          target_velocity_ = 0.0;
        }
        break;

      case CascadeState::Holding:
        if (stick_active) {
          integral_vel_ = 0.0;
          cascade_state_ = CascadeState::Driving;
          target_velocity_ = clampValue(cmd_velocity, -max_vel_cmd_, max_vel_cmd_);
        } else {
          pos_error_ = target_enc_pos_ - enc_pos;
          target_velocity_ =
            clampValue(kp_pos_ * pos_error_, -max_vel_cmd_, max_vel_cmd_);
        }
        break;
    }

    // =====================================================================
    //  LAYER 2 (MIDDLE): velocity PI -> tilt bias (degrees)
    // =====================================================================
    vel_error_ = target_velocity_ - vel_current_;

    if (!disable_integral_) {
      integral_vel_ += ki_vel_ * vel_error_ * dt;
      integral_vel_ = clampValue(integral_vel_, -tilt_bias_limit_, tilt_bias_limit_);
    } else {
      integral_vel_ = 0.0;
    }

    tilt_bias_ = (kp_vel_ * vel_error_) + integral_vel_;
    tilt_bias_ = clampValue(tilt_bias_, -tilt_bias_limit_, tilt_bias_limit_);

    // =====================================================================
    //  LAYER 1 (INNER): balance PID
    // =====================================================================
    const double target_angle = base_target_angle_ + tilt_bias_ + gimbal_offset_;
    const double error = target_angle - pitch_;

    if (!disable_integral_) {
      // Unscaled accumulator, exactly as the firmware does it. ki_ is applied
      // below, which is why ki = 670 is reasonable here. There is deliberately
      // NO clamp: this reproduces the firmware's real windup behaviour.
      integral_ += error * dt;
    } else {
      integral_ = 0.0;
    }

    const double derivative = -gyro_rate_;   // derivative on measurement
    output_ = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

    // --- Mixing and output ------------------------------------------------
    // effort_sign_ carries the firmware's negation (default -1.0).
    double left_pwm = (effort_sign_ * output_) + turn_bias;
    double right_pwm = (effort_sign_ * output_) - turn_bias;

    left_pwm = clampValue(left_pwm, -pwm_max_, pwm_max_);
    right_pwm = clampValue(right_pwm, -pwm_max_, pwm_max_);

    last_left_pwm_ = left_pwm;
    last_right_pwm_ = right_pwm;

    publishWheelEffort(pwmToTorque(left_pwm, left_wheel_vel_rad_),
                       pwmToTorque(right_pwm, right_wheel_vel_rad_));

    publishTelemetry(dt);
  }

  void publishWheelEffort(double left_torque, double right_torque)
  {
    std_msgs::msg::Float64MultiArray msg;
    // Order matches wheel_effort_controller's joint list: [left, right].
    msg.data = {left_torque, right_torque};
    wheel_cmd_pub_->publish(msg);
  }

  // =========================================================================
  //  Telemetry
  //
  //  Published at the full control rate so oscillations are captured without
  //  aliasing. Field order is fixed; the README and the GUI bridge both
  //  depend on it.
  //
  //   0  pitch (deg)              8  vel_error (counts/s)
  //   1  target_angle (deg)       9  pos_error (counts)
  //   2  error (deg)             10  cascade_state (0/1/2)
  //   3  pid_output (pwm)        11  motors_enabled (0/1)
  //   4  integral                12  safety_latched (0/1)
  //   5  tilt_bias (deg)         13  left_pwm
  //   6  gyro_rate (deg/s)       14  right_pwm
  //   7  vel_current (counts/s)  15  dt (s)
  // =========================================================================
  void publishTelemetry(double dt)
  {
    if (telemetry_rate_ > 0.0) {
      const double interval = 1.0 / telemetry_rate_;
      const rclcpp::Time current = now();
      if (last_telemetry_time_.nanoseconds() > 0 &&
          (current - last_telemetry_time_).seconds() < interval)
      {
        return;
      }
      last_telemetry_time_ = current;
    }

    const double target_angle = base_target_angle_ + tilt_bias_ + gimbal_offset_;

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      pitch_,
      target_angle,
      target_angle - pitch_,
      output_,
      integral_,
      tilt_bias_,
      gyro_rate_,
      vel_current_,
      vel_error_,
      pos_error_,
      static_cast<double>(static_cast<int>(cascade_state_)),
      motors_enabled_ ? 1.0 : 0.0,
      safety_latched_ ? 1.0 : 0.0,
      last_left_pwm_,
      last_right_pwm_,
      dt
    };
    telemetry_pub_->publish(msg);
  }

  // =========================================================================
  //  Members
  // =========================================================================

  // Publishers / subscribers / services
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Gains and configuration
  double control_rate_{100.0};
  double kp_{95.0}, ki_{670.0}, kd_{2.0};
  double alpha_{0.96};
  double base_target_angle_{0.0};
  double max_safe_tilt_{25.0};
  double kp_vel_{0.02}, ki_vel_{0.001}, tilt_bias_limit_{5.0};
  double kp_pos_{0.5}, max_vel_cmd_{800.0}, hold_vel_threshold_{20.0};
  double vel_alpha_{0.85};
  double counts_per_rev_{1320.0}, wheel_radius_{0.033};
  double pwm_max_{255.0}, stall_torque_{0.15}, no_load_speed_{31.4};
  bool use_motor_model_{true};
  double effort_sign_{-1.0};
  double telemetry_rate_{100.0}, cmd_vel_timeout_{0.5};
  std::string left_wheel_name_{"left_wheel_joint"};
  std::string right_wheel_name_{"right_wheel_joint"};

  // Attitude
  double pitch_{0.0};
  double accel_pitch_raw_{0.0};
  double pitch_offset_{0.0};
  double gyro_rate_{0.0};

  // Wheel feedback
  double left_wheel_pos_rad_{0.0}, right_wheel_pos_rad_{0.0};
  double left_wheel_vel_rad_{0.0}, right_wheel_vel_rad_{0.0};
  int left_wheel_idx_{-1}, right_wheel_idx_{-1};

  // Cascade state
  double integral_{0.0}, integral_vel_{0.0}, tilt_bias_{0.0};
  double vel_current_{0.0}, target_velocity_{0.0}, target_enc_pos_{0.0};
  double pos_error_{0.0}, vel_error_{0.0}, output_{0.0};
  double last_left_pwm_{0.0}, last_right_pwm_{0.0};
  CascadeState cascade_state_{CascadeState::Holding};

  // Commands
  double cmd_velocity_counts_{0.0};
  double cmd_turn_bias_{0.0};
  double gimbal_offset_{0.0};

  // Flags and timing
  bool motors_enabled_{false};
  bool safety_latched_{false};
  bool disable_integral_{false};
  bool imu_ready_{false};
  bool joints_ready_{false};
  bool have_last_step_{false};
  rclcpp::Time last_step_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_telemetry_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalancePoint>());
  rclcpp::shutdown();
  return 0;
}

// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Keyboard jog teleop. This node sits where joy_node + teleop would: it does
// not link WMX3 and only publishes wmx/axes/start_jog. wmx_core_motion_node owns the
// dead-man (jog_timeout_ms), so this node just has to keep publishing while a
// key is held and go quiet when it is released.
//
// A terminal never reports key releases, so a key is considered held while
// auto-repeat characters keep arriving, and considered released once none has
// arrived for a grace period.
//
// Auto-repeat delivers one character, then nothing until the repeat delay has
// elapsed, then a steady stream. Treating that first gap as a release makes the
// axis stutter once at the start of every press, so the grace period is
// initial_grace_s until the stream is seen and hold_grace_s afterwards. The
// repeat delay belongs to whichever machine the operator types on, which over
// ssh is not this one, so it is measured on the first press rather than assumed.

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "wmx_r2_message/msg/axes_velocity.hpp"

namespace
{
constexpr char kJogNegative = 'a';
constexpr char kJogPositive = 'd';
constexpr char kQuit = 'q';

// Rejects a nonsense measurement and caps how long a released key can keep the
// axis moving, whatever the keyboard reports.
constexpr double kMaxInitialGraceS = 1.0;

// Puts the terminal in non-canonical no-echo mode and restores it on scope
// exit, so the shell is left usable whichever way the node terminates.
class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    tcgetattr(STDIN_FILENO, &saved_);
    termios raw = saved_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~TerminalRawMode()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
  }

  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode & operator=(const TerminalRawMode &) = delete;

private:
  termios saved_;
};

}  // namespace

class JogKeyboardNode : public rclcpp::Node
{
public:
  JogKeyboardNode()
  : Node("jog_keyboard_node")
  {
    axis_ = this->declare_parameter("axis", 0);
    velocity_ = this->declare_parameter("velocity", 10000.0);
    acc_ = this->declare_parameter("acc", 100000.0);
    dec_ = this->declare_parameter("dec", 100000.0);
    holdGraceS_ = this->declare_parameter("hold_grace_s", 0.1);
    initialGraceS_ = this->declare_parameter("initial_grace_s", 0.8);
    graceMarginS_ = this->declare_parameter("grace_margin_s", 0.06);
    const double publishRate = this->declare_parameter("publish_rate", 20.0);

    jogPub_ = this->create_publisher<wmx_r2_message::msg::AxesVelocity>("wmx/axes/start_jog", 1);
    lastKeyTime_ = this->now();
    firstKeyTime_ = lastKeyTime_;

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publishRate),
      std::bind(&JogKeyboardNode::step, this));

    RCLCPP_INFO(
      this->get_logger(),
      "axis %d ready. '%c' = jog negative, '%c' = jog positive, '%c' = quit.",
      axis_, kJogNegative, kJogPositive, kQuit);

    if (!::isatty(STDIN_FILENO)) {
      RCLCPP_WARN(
        this->get_logger(),
        "stdin is not a terminal, so no keys will be read. "
        "Run this node from a terminal instead of a launch file.");
    }
  }

private:
  // select() first so this never blocks when stdin is a pipe rather than a tty
  // (for example when the node is started from a launch file). The per-tick cap
  // keeps this bounded: without it a writer that fills stdin as fast as we read
  // would keep the timer callback from ever returning.
  static std::vector<char> drainStdin()
  {
    constexpr size_t kMaxKeysPerTick = 64;

    std::vector<char> keys;
    while (keys.size() < kMaxKeysPerTick) {
      fd_set readFds;
      FD_ZERO(&readFds);
      FD_SET(STDIN_FILENO, &readFds);
      timeval noWait{0, 0};

      if (::select(STDIN_FILENO + 1, &readFds, nullptr, nullptr, &noWait) <= 0) {
        break;
      }

      char c;
      if (::read(STDIN_FILENO, &c, 1) != 1) {
        break;
      }
      keys.push_back(c);
    }
    return keys;
  }

  // Keep the initial grace just above the observed repeat delay: long enough not
  // to mistake the pre-repeat gap for a release, short enough that a released
  // key stops the axis promptly.
  void learnRepeatDelay(double delayS)
  {
    if (delayS <= 0.0 || delayS > kMaxInitialGraceS) {
      return;
    }

    const double grace = std::min(delayS + graceMarginS_, kMaxInitialGraceS);
    if (std::abs(grace - initialGraceS_) < 0.02) {
      return;
    }

    initialGraceS_ = grace;
    RCLCPP_INFO(
      this->get_logger(),
      "Measured a %.0f ms key repeat delay, so a released key now stops the axis within "
      "%.2f s. Shorten the repeat delay on the machine you type on to tighten this.",
      delayS * 1000.0, initialGraceS_);
  }

  void step()
  {
    const std::vector<char> keys = drainStdin();

    // Ctrl-C is left to the signal handler: ISIG is still set on the terminal,
    // so it never arrives here as a character.
    if (std::find(keys.begin(), keys.end(), kQuit) != keys.end()) {
      rclcpp::shutdown();
      return;
    }

    // Most recent direction key in this batch wins.
    double pressed = 0.0;
    for (auto it = keys.rbegin(); it != keys.rend(); ++it) {
      if (*it == kJogNegative) {
        pressed = -1.0;
        break;
      }
      if (*it == kJogPositive) {
        pressed = 1.0;
        break;
      }
    }

    const rclcpp::Time now = this->now();
    if (pressed != 0.0) {
      if (pressed != direction_) {
        repeatSeen_ = false;
        firstKeyTime_ = now;
      } else if (!repeatSeen_) {
        // First repeated batch for this key: the gap since the initial character
        // is this keyboard's repeat delay. Over ssh that delay belongs to the
        // client machine and cannot be read with xset, so measure it instead of
        // guessing, and keep the initial grace just above it.
        repeatSeen_ = true;
        learnRepeatDelay((now - firstKeyTime_).seconds());
      }
      direction_ = pressed;
      lastKeyTime_ = now;
    } else {
      const double grace = repeatSeen_ ? holdGraceS_ : initialGraceS_;
      if ((now - lastKeyTime_).seconds() > grace) {
        if (direction_ != 0.0) {
          // Say "stop" rather than falling silent: the driver stops on a zero
          // velocity straight away, so the release does not have to wait out the
          // dead-man timeout. That timeout stays as the backstop for this node
          // dying or the network dropping.
          publishJog(0.0);
        }
        direction_ = 0.0;
        repeatSeen_ = false;
      }
    }

    if (direction_ == 0.0) {
      return;
    }
    publishJog(direction_);
  }

  void publishJog(double direction)
  {
    wmx_r2_message::msg::AxesVelocity msg;
    msg.indices = {axis_};
    msg.velocities = {velocity_ * direction};
    msg.accelerations = {acc_};
    msg.decelerations = {dec_};
    jogPub_->publish(msg);
  }

  int axis_;
  double velocity_;
  double acc_;
  double dec_;
  double holdGraceS_;
  double initialGraceS_;
  double graceMarginS_;
  double direction_ = 0.0;
  bool repeatSeen_ = false;
  rclcpp::Time lastKeyTime_;
  rclcpp::Time firstKeyTime_;

  rclcpp::Publisher<wmx_r2_message::msg::AxesVelocity>::SharedPtr jogPub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  TerminalRawMode rawMode;
  rclcpp::spin(std::make_shared<JogKeyboardNode>());
  rclcpp::shutdown();
  return 0;
}

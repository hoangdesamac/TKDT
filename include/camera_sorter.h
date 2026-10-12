#pragma once

#include <stdint.h>

// One mango at a time: IR1/camera, then IR2/dark chamber.
// Hardware-independent; all timing is non-blocking.
class CameraSorter {
 public:
  enum class State : uint8_t {
    Stopped, Moving, CameraSettling, WaitingPi, GateSettling,
    Rejecting, Returning, Passing, MovingToDark, WaitingDarkPi,
    Gate2Settling, Rejecting2, Returning2, SweetPassing, Fault
  };
  enum class Fault : uint8_t {
    None, PiTimeout, Ir1Timeout, ServoMissing, DarkPiTimeout,
    Ir2Timeout, Ir2ArrivalTimeout
  };

  static constexpr uint32_t kCameraSettlingMs = 200;
  static constexpr uint32_t kPiTimeoutMs = 30000;
  static constexpr uint32_t kServoSettlingMs = 300;
  static constexpr uint32_t kIr1ClearTimeoutMs = 10000;
  static constexpr uint32_t kIr2ClearTimeoutMs = 10000;
  static constexpr uint32_t kIr2ArrivalTimeoutMs = 30000;

  void setRunning(bool run, uint32_t now) {
    if (!run) {
      fault_ = Fault::None;
      cameraPending_ = false;
      darkPending_ = false;
      enter(State::Stopped, now);
    } else if (state_ == State::Stopped ||
               (state_ == State::Fault && fault_ != Fault::ServoMissing)) {
      fault_ = Fault::None;
      cameraPending_ = false;
      darkPending_ = false;
      result_ = -1;
      darkResult_ = -1;
      enter(State::Moving, now);
    }
    // Repeated START preserves the current camera/gating operation.
    // Explicit START can recover a timeout, matching the CYD toggle button.
  }

  bool acceptResult(uint8_t result, uint32_t now) {
    const bool dark = state_ == State::WaitingDarkPi;
    if ((!dark && state_ != State::WaitingPi) || result > 1) return false;
    if (elapsed(now) >= kPiTimeoutMs) {
      fail(dark ? Fault::DarkPiTimeout : Fault::PiTimeout, now);
      return false;
    }
    if (dark) {
      darkResult_ = result;
      darkPending_ = false;
      enter(result == 0 ? State::Gate2Settling : State::SweetPassing, now);
    } else {
      result_ = result;
      cameraPending_ = false;
      enter(result == 0 ? State::GateSettling : State::Passing, now);
    }
    return true;
  }

  void tick(uint32_t now, bool ir1Blocked, bool ir2Blocked = false) {
    switch (state_) {
      case State::Moving:
        // Also stop for a mango already at IR2 when START is pressed.
        if (ir2Blocked) {
          result_ = -1;
          waitForDarkResult(now);
        } else if (ir1Blocked) {
          result_ = -1;
          darkResult_ = -1;
          enter(State::CameraSettling, now);
        }
        break;
      case State::CameraSettling:
        if (elapsed(now) >= kCameraSettlingMs) {
          enter(State::WaitingPi, now);
          cameraPending_ = true;
        }
        break;
      case State::WaitingPi:
        if (elapsed(now) >= kPiTimeoutMs) fail(Fault::PiTimeout, now);
        break;
      case State::GateSettling:
        if (elapsed(now) >= kServoSettlingMs) enter(State::Rejecting, now);
        break;
      case State::Rejecting:
        if (!ir1Blocked) enter(State::Returning, now);
        else if (elapsed(now) >= kIr1ClearTimeoutMs)
          fail(Fault::Ir1Timeout, now);
        break;
      case State::Passing:
        if (ir2Blocked) waitForDarkResult(now);
        else if (!ir1Blocked) enter(State::MovingToDark, now);
        else if (elapsed(now) >= kIr1ClearTimeoutMs)
          fail(Fault::Ir1Timeout, now);
        break;
      case State::Returning:
        if (elapsed(now) >= kServoSettlingMs) enter(State::Moving, now);
        break;
      case State::MovingToDark:
        if (ir2Blocked) waitForDarkResult(now);
        else if (elapsed(now) >= kIr2ArrivalTimeoutMs)
          fail(Fault::Ir2ArrivalTimeout, now);
        break;
      case State::WaitingDarkPi:
        if (elapsed(now) >= kPiTimeoutMs) fail(Fault::DarkPiTimeout, now);
        break;
      case State::Gate2Settling:
        if (elapsed(now) >= kServoSettlingMs) enter(State::Rejecting2, now);
        break;
      case State::Rejecting2:
        if (!ir2Blocked) enter(State::Returning2, now);
        else if (elapsed(now) >= kIr2ClearTimeoutMs)
          fail(Fault::Ir2Timeout, now);
        break;
      case State::Returning2:
        if (elapsed(now) >= kServoSettlingMs) enter(State::Moving, now);
        break;
      case State::SweetPassing:
        if (!ir2Blocked) enter(State::Moving, now);
        else if (elapsed(now) >= kIr2ClearTimeoutMs)
          fail(Fault::Ir2Timeout, now);
        break;
      default:
        break;
    }
  }

  void fail(Fault reason, uint32_t now) {
    fault_ = reason;
    cameraPending_ = false;
    darkPending_ = false;
    enter(State::Fault, now);
  }

  bool takeCameraRequest() {
    const bool pending = cameraPending_;
    cameraPending_ = false;
    return pending;
  }

  bool beltOn() const {
    return state_ == State::Moving || state_ == State::Rejecting ||
           state_ == State::Passing || state_ == State::MovingToDark ||
           state_ == State::Rejecting2 || state_ == State::SweetPassing;
  }
  bool gateExtended() const {
    return state_ == State::GateSettling || state_ == State::Rejecting;
  }
  bool gate2Extended() const {
    return state_ == State::Gate2Settling || state_ == State::Rejecting2;
  }
  bool takeDarkRequest() {
    const bool pending = darkPending_;
    darkPending_ = false;
    return pending;
  }
  bool running() const { return state_ != State::Stopped && state_ != State::Fault; }
  State state() const { return state_; }
  Fault fault() const { return fault_; }
  int8_t result() const { return result_; }
  int8_t darkResult() const { return darkResult_; }

 private:
  void waitForDarkResult(uint32_t now) {
    darkResult_ = -1;
    enter(State::WaitingDarkPi, now);
    darkPending_ = true;
  }
  uint32_t elapsed(uint32_t now) const { return now - stateSince_; }
  void enter(State next, uint32_t now) {
    state_ = next;
    stateSince_ = now;
  }
  State state_ = State::Stopped;
  Fault fault_ = Fault::None;
  uint32_t stateSince_ = 0;
  bool cameraPending_ = false;
  bool darkPending_ = false;
  int8_t result_ = -1;
  int8_t darkResult_ = -1;
};

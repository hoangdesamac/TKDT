// Host test: g++ -std=c++11 -Iinclude test/camera_sorter_test.cpp -o sorter_test
#include "camera_sorter.h"
#include <cassert>
#include <cstdio>

using State = CameraSorter::State;

CameraSorter waitingForPi(uint32_t start = 0) {
  CameraSorter sorter;
  sorter.setRunning(true, start);
  assert(sorter.beltOn());
  sorter.tick(start + 30, true);
  assert(sorter.state() == State::CameraSettling && !sorter.beltOn());
  assert(!sorter.takeCameraRequest());
  sorter.tick(start + 30 + CameraSorter::kCameraSettlingMs, true);
  assert(sorter.state() == State::WaitingPi && !sorter.beltOn());
  assert(sorter.takeCameraRequest());
  assert(!sorter.takeCameraRequest());
  return sorter;
}

void checkStop(CameraSorter sorter, uint32_t now) {
  sorter.setRunning(false, now);
  assert(sorter.state() == State::Stopped);
  assert(!sorter.running() && !sorter.beltOn() && !sorter.gateExtended());
  assert(!sorter.gate2Extended() && !sorter.takeDarkRequest());
  assert(!sorter.takeCameraRequest());
  assert(!sorter.acceptResult(0, now + 1));
  sorter.tick(now + 100000, true, true);
  assert(sorter.state() == State::Stopped);
}

int main() {
  CameraSorter stopped;
  assert(!stopped.acceptResult(0, 0));
  stopped.tick(100, true);
  assert(stopped.state() == State::Stopped && !stopped.takeCameraRequest());

  // Remain stopped at IR1 for the entire camera operation, even if IR1 clears.
  auto cameraWait = waitingForPi();
  cameraWait.tick(1000, true);
  cameraWait.tick(2000, false);
  cameraWait.setRunning(true, 2001);
  assert(cameraWait.state() == State::WaitingPi && !cameraWait.beltOn());
  assert(!cameraWait.gateExtended() && !cameraWait.takeCameraRequest());

  auto reject = waitingForPi();
  assert(!reject.acceptResult(2, 231));
  reject.setRunning(true, 231);  // Duplicate START must not restart the belt.
  assert(reject.state() == State::WaitingPi && !reject.beltOn());
  assert(reject.acceptResult(0, 232));
  assert(reject.state() == State::GateSettling);
  assert(!reject.beltOn() && reject.gateExtended());
  assert(!reject.acceptResult(1, 233));  // Duplicate/late reply is ignored.
  checkStop(reject, 234);
  reject.tick(531, true);
  assert(!reject.beltOn());
  reject.tick(532, true);
  assert(reject.state() == State::Rejecting);
  assert(reject.beltOn() && reject.gateExtended());
  checkStop(reject, 533);
  reject.tick(600, true);
  assert(reject.state() == State::Rejecting);
  reject.tick(601, false);
  assert(reject.state() == State::Returning);
  assert(!reject.beltOn() && !reject.gateExtended());
  checkStop(reject, 602);
  reject.tick(900, false);
  assert(reject.state() == State::Returning);
  reject.tick(901, false);
  assert(reject.state() == State::Moving && reject.beltOn());
  reject.tick(902, false);
  assert(!reject.takeCameraRequest());
  reject.tick(903, true);  // A new mango can start a new camera request.
  assert(reject.state() == State::CameraSettling);

  auto pass = waitingForPi();
  assert(pass.acceptResult(1, 232));
  assert(pass.state() == State::Passing && pass.beltOn());
  assert(!pass.gateExtended());
  pass.tick(3000, true);
  assert(pass.state() == State::Passing && !pass.takeCameraRequest());
  checkStop(pass, 3001);
  pass.tick(3002, false);
  assert(pass.state() == State::MovingToDark && pass.beltOn());
  checkStop(pass, 3003);
  pass.tick(3003, true);
  assert(pass.state() == State::MovingToDark && !pass.takeCameraRequest());
  assert(!pass.acceptResult(0, 3004));  // Cannot use old camera replies at IR2.
  pass.tick(3100, false, true);
  assert(pass.state() == State::WaitingDarkPi && !pass.beltOn());
  assert(pass.takeDarkRequest() && !pass.takeDarkRequest());
  assert(pass.result() == 1 && pass.darkResult() == -1);
  checkStop(pass, 3101);
  pass.setRunning(true, 3101);
  pass.tick(3102, false, false);  // Clearing IR2 does not cancel the model wait.
  assert(pass.state() == State::WaitingDarkPi && !pass.beltOn());
  assert(!pass.takeDarkRequest() && !pass.acceptResult(2, 3103));

  auto sour = pass;
  assert(sour.acceptResult(0, 3200));
  assert(sour.darkResult() == 0 && sour.result() == 1);
  assert(sour.state() == State::Gate2Settling);
  assert(!sour.beltOn() && sour.gate2Extended() && !sour.gateExtended());
  assert(!sour.acceptResult(1, 3201));
  checkStop(sour, 3202);
  sour.tick(3499, false, true);
  assert(sour.state() == State::Gate2Settling && !sour.beltOn());
  sour.tick(3500, false, true);
  assert(sour.state() == State::Rejecting2 && sour.beltOn() && sour.gate2Extended());
  checkStop(sour, 3501);
  sour.tick(3600, false, true);  // IR1 clear must not retract servo2.
  assert(sour.state() == State::Rejecting2 && sour.gate2Extended());
  sour.tick(3700, false, false);
  assert(sour.state() == State::Returning2 && !sour.beltOn() && !sour.gate2Extended());
  checkStop(sour, 3701);
  sour.tick(3999, false, false);
  assert(sour.state() == State::Returning2);
  sour.tick(4000, false, false);
  assert(sour.state() == State::Moving && sour.beltOn());
  sour.tick(4001, true, false);
  assert(sour.state() == State::CameraSettling && sour.darkResult() == -1);

  auto sweet = pass;
  assert(sweet.acceptResult(1, 3200));
  assert(sweet.state() == State::SweetPassing && sweet.beltOn());
  assert(!sweet.gateExtended() && !sweet.gate2Extended() && sweet.darkResult() == 1);
  checkStop(sweet, 3201);
  sweet.tick(3300, false, true);
  assert(sweet.state() == State::SweetPassing && sweet.beltOn());
  sweet.tick(3400, false, false);
  assert(sweet.state() == State::Moving && sweet.beltOn());

  auto darkTimeout = pass;
  darkTimeout.tick(3100 + CameraSorter::kPiTimeoutMs, false, true);
  assert(darkTimeout.fault() == CameraSorter::Fault::DarkPiTimeout);
  assert(!darkTimeout.beltOn() && !darkTimeout.running());
  auto darkLate = pass;
  assert(!darkLate.acceptResult(0, 3100 + CameraSorter::kPiTimeoutMs));
  assert(darkLate.fault() == CameraSorter::Fault::DarkPiTimeout);
  for (uint8_t result = 0; result <= 1; ++result) {
    auto jam2 = pass;
    assert(jam2.acceptResult(result, 3200));
    jam2.tick(3500, false, true);
    jam2.tick(3500 + CameraSorter::kIr2ClearTimeoutMs, false, true);
    assert(jam2.fault() == CameraSorter::Fault::Ir2Timeout);
    assert(!jam2.beltOn() && !jam2.gate2Extended());
  }
  auto noIr2 = waitingForPi();
  assert(noIr2.acceptResult(1, 232));
  noIr2.tick(233, false, false);
  noIr2.tick(233 + CameraSorter::kIr2ArrivalTimeoutMs, false, false);
  assert(noIr2.fault() == CameraSorter::Fault::Ir2ArrivalTimeout && !noIr2.beltOn());

  // Sensors can overlap; IR2 must stop the belt even before IR1 clears.
  auto overlap = waitingForPi();
  assert(overlap.acceptResult(1, 232));
  overlap.tick(233, true, true);
  assert(overlap.state() == State::WaitingDarkPi && !overlap.beltOn());
  assert(overlap.takeDarkRequest());
  CameraSorter alreadyAtIr2;
  alreadyAtIr2.setRunning(true, 0);
  alreadyAtIr2.tick(1, false, true);
  assert(alreadyAtIr2.state() == State::WaitingDarkPi && !alreadyAtIr2.beltOn());
  assert(alreadyAtIr2.takeDarkRequest());

  auto timeout = waitingForPi();
  timeout.tick(230 + CameraSorter::kPiTimeoutMs, true);
  assert(timeout.fault() == CameraSorter::Fault::PiTimeout);
  assert(!timeout.beltOn() && !timeout.running());
  assert(!timeout.acceptResult(0, 30231));
  timeout.setRunning(true, 30232);
  assert(timeout.state() == State::Moving);
  assert(timeout.fault() == CameraSorter::Fault::None);
  timeout.setRunning(false, 30233);
  timeout.setRunning(true, 30234);
  assert(timeout.state() == State::Moving);

  auto lateReply = waitingForPi();
  assert(!lateReply.acceptResult(1, 230 + CameraSorter::kPiTimeoutMs));
  assert(lateReply.state() == State::Fault && !lateReply.beltOn());

  for (uint8_t result = 0; result <= 1; ++result) {
    auto jam = waitingForPi();
    assert(jam.acceptResult(result, 232));
    jam.tick(532, true);
    jam.tick(532 + CameraSorter::kIr1ClearTimeoutMs, true);
    assert(jam.fault() == CameraSorter::Fault::Ir1Timeout);
    assert(!jam.beltOn() && !jam.gateExtended());
  }

  auto stopWaiting = waitingForPi();
  checkStop(stopWaiting, 231);
  CameraSorter moving;
  moving.setRunning(true, 0);
  checkStop(moving, 1);
  moving.tick(30, true);
  checkStop(moving, 31);

  CameraSorter missingServo;
  missingServo.fail(CameraSorter::Fault::ServoMissing, 0);
  missingServo.setRunning(true, 1);
  assert(missingServo.state() == State::Fault && !missingServo.beltOn());

  // millis() wrapping after 49 days must not break deadlines.
  auto wrapped = waitingForPi(UINT32_MAX - 100);
  assert(wrapped.acceptResult(0, 131));
  wrapped.tick(431, true);
  assert(wrapped.state() == State::Rejecting && wrapped.beltOn());

  CameraSorter wrappedDark;
  wrappedDark.setRunning(true, UINT32_MAX - 100);
  wrappedDark.tick(UINT32_MAX - 99, false, true);
  assert(wrappedDark.acceptResult(0, UINT32_MAX - 98));
  wrappedDark.tick(201, false, true);
  assert(wrappedDark.state() == State::Rejecting2 && wrappedDark.beltOn());

  std::puts("camera_sorter: PASS (IR1/IR2, both servos, STOP, replies, timeouts, wrap)");
}

#
#   Developer : Coen Smeets (Coen@vectioneer.com)
#   All rights reserved. Copyright (c) 2025 VECTIONEER.
#

"""
절대 위치 모드 - hostInJointPosition2 4축 동시 제어
[Machinecontrol] -> [Axescontrol]
[Joint(ch)] -> [Axes(ch,rad) -> Actuator(rad) -> motor(ticks)]

읽기 경로:
  root/AxesControl/axesPositionsActual/ch0~ (rad) - 4축 실제 위치

쓰기 경로:
  root/AxesControl/axesPositionsInput/ch0~ (rad) - 위치제어모드 -> PVA limit문제, Setpoint jump 문제 발생 - machine control link 비활성화 필요
  root/MachineControl/hostInJointPosition2/ch0~ (rad) - 위치제어모드 -> signalgenerator link 해제, mode SPG로 변경 후 제어가능. 
  root/MachineControl/hostInJointadditivePosition2/ch0~ (rad) - 위치제어모드 -> Jogmode에서도 가능함.

"""


import logging
import signal
import time
import math
import os
logging.basicConfig(level=logging.INFO)

from src.mcx_client_app import McxClientApp, McxClientAppConfiguration

# ── 설정 상수 ──────────────────────────────────────────────────────────────────
NUM_AXES     = 4
NUM_CH       = 6    # hostInJointPosition2 전체 채널 수
ACTUAL_PATH  = "root/AxesControl/axesPositionsActual"  # 부모 경로, value[0~3] 으로 인덱싱
POS_CMD_PATH = "root/MachineControl/hostInJointPosition2"

STATE_CMD_PATH = "root/Logic/stateCommand"
STATE_PATH     = "root/Logic/state"
ENGAGE_CMD     = 2
ENGAGED_STATE  = 4
ENGAGE_TIMEOUT = 10.0

HOME_MODE_PATH = "root/UserParameters/homemode"
JUMP_MODE_PATH = "root/UserParameters/jumpmode"

HOME_THRESHOLD_RAD = math.radians(0.0)
HOME_MAX_VEL       = math.radians(20.0)
HOME_MAX_ACC       = math.radians(10.0)
HOME_DT            = 0.005

AMPLITUDE_RAD = math.radians(10.0)
FREQUENCY_HZ  = 0.1
CYCLE_TIME    = 0.001
# ───────────────────────────────────────────────────────────────────────────────


class CspClientAppV5(McxClientApp):

    def __init__(self, options: McxClientAppConfiguration):
        super().__init__(options)
        self._actual_pos:     list[float] = [0.0] * NUM_AXES  # 구독 캐시 (실시간)
        self._hold_pos:       list[float] = [0.0] * NUM_AXES  # standby 유지 target
        self._home_start_pos: list[float] = [0.0] * NUM_AXES  # homemode 버튼 누른 시점 actual
        self.ready:           bool        = False
        self._home_requested: bool        = False
        self._jump_requested: bool        = False

    # ── 위치 명령 전송 ────────────────────────────────────────────────────────
    def _set_positions(self, positions: list[float], blocking: bool = False):
        # 6채널 배열로 전송, ch0~ch3만 제어하고 ch4~ch5는 0으로 패딩
        cmd = list(positions) + [0.0] * (NUM_CH - NUM_AXES)
        future = self.req.setParameter(POS_CMD_PATH, cmd)
        if blocking:
            future.get()

    # ── Engage ────────────────────────────────────────────────────────────────
    def _engage(self) -> bool:
        try:
            state = self.req.getParameter(STATE_PATH).get()
            if state and state.value and state.value[0] == ENGAGED_STATE:
                logging.info("이미 Engaged 상태.")
                return True
        except Exception:
            pass

        logging.info("Engage 명령 전송...")
        self.req.setParameter(STATE_CMD_PATH, [ENGAGE_CMD]).get()

        deadline = time.time() + ENGAGE_TIMEOUT
        while time.time() < deadline:
            try:
                state = self.req.getParameter(STATE_PATH).get()
                if state and state.value and state.value[0] == ENGAGED_STATE:
                    logging.info("Engage 완료.")
                    return True
            except Exception:
                pass
            time.sleep(0.1)

        logging.error("Engage 타임아웃!")
        return False

    # ── 사다리꼴 프로파일 ─────────────────────────────────────────────────────
    def _trapezoid_profile(self, dist: float, max_vel: float, max_acc: float, dt: float) -> list:
        """0 → dist 누적 위치 리스트 반환."""
        d_acc = max_vel ** 2 / (2.0 * max_acc)
        if 2.0 * d_acc >= dist:
            v_peak  = math.sqrt(max_acc * dist)
            t_acc   = v_peak / max_acc
            t_total = 2.0 * t_acc
        else:
            t_acc   = max_vel / max_acc
            t_const = (dist - 2.0 * d_acc) / max_vel
            t_total = 2.0 * t_acc + t_const
            v_peak  = max_vel

        n_steps   = max(1, int(t_total / dt))
        positions = []
        for k in range(1, n_steps + 1):
            t = t_total * k / n_steps
            if 2.0 * d_acc >= dist:
                t_acc_loc = v_peak / max_acc
                if t <= t_acc_loc:
                    p = 0.5 * max_acc * t ** 2
                else:
                    t2 = t - t_acc_loc
                    p  = 0.5 * max_acc * t_acc_loc ** 2 + v_peak * t2 - 0.5 * max_acc * t2 ** 2
            else:
                if t <= t_acc:
                    p = 0.5 * max_acc * t ** 2
                elif t <= t_acc + t_const:
                    p = d_acc + max_vel * (t - t_acc)
                else:
                    t3 = t - t_acc - t_const
                    p  = d_acc + max_vel * t_const + max_vel * t3 - 0.5 * max_acc * t3 ** 2
            positions.append(min(p, dist))
        return positions

    # ── 홈 복귀 ───────────────────────────────────────────────────────────────
    def move_to_home(self) -> None:
        """homemode 버튼 누른 시점 actual_pos → 0° 사다리꼴 궤적 (blocking)."""
        start_pos = list(self._home_start_pos)

        logging.info(
            "홈 복귀 시작: "
            + ", ".join(f"축{i+1}={math.degrees(start_pos[i]):+.1f}°→0°" for i in range(NUM_AXES))
        )

        if all(abs(p) <= HOME_THRESHOLD_RAD for p in start_pos):
            logging.info("이미 홈 위치 — 스킵.")
            return

        max_dist = max(abs(p) for p in start_pos)
        profile  = self._trapezoid_profile(max_dist, HOME_MAX_VEL, HOME_MAX_ACC, HOME_DT)
        n_steps  = len(profile)

        t0 = time.monotonic()
        for k, p in enumerate(profile):
            target_pos = [start_pos[i] * (1.0 - p / max_dist) for i in range(NUM_AXES)]
            self._set_positions(target_pos)

            sleep_t = t0 + (k + 1) * HOME_DT - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)

            if any(abs(self._actual_pos[i]) > HOME_THRESHOLD_RAD for i in range(NUM_AXES)):
                logging.warning(
                    f"[{k+1}/{n_steps}] 홈 거리: "
                    + ", ".join(f"축{i+1}={math.degrees(self._actual_pos[i]):+.2f}°" for i in range(NUM_AXES))
                )

        self._set_positions([0.0] * NUM_AXES, blocking=True)
        final_errors = [abs(self._actual_pos[i]) for i in range(NUM_AXES)]
        if all(e <= HOME_THRESHOLD_RAD for e in final_errors):
            logging.info(f"홈 복귀 완료 ({n_steps * HOME_DT:.2f}s)")
        else:
            logging.warning(
                "홈 복귀 완료 (오차 있음): "
                + ", ".join(f"축{i+1}={math.degrees(final_errors[i]):.2f}°" for i in range(NUM_AXES))
            )

        self._hold_pos = [0.0] * NUM_AXES

    # ── 점프 궤적 ─────────────────────────────────────────────────────────────
    def _run_jump(self) -> None:
        """현재 actual_pos 기준 ±AMPLITUDE 사인파 1사이클."""
        start_pos = list(self._actual_pos)

        t_total = 1.0 / FREQUENCY_HZ
        n_steps = int(t_total / CYCLE_TIME)
        t0 = time.monotonic()

        for k in range(n_steps):
            sin_val = AMPLITUDE_RAD * math.sin(2.0 * math.pi * FREQUENCY_HZ * (k * CYCLE_TIME))
            target_pos = [start_pos[i] + sin_val for i in range(NUM_AXES)]
            self._set_positions(target_pos)

            sleep_t = t0 + (k + 1) * CYCLE_TIME - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)

        self._set_positions(start_pos, blocking=True)

    # ── 초기화 ────────────────────────────────────────────────────────────────
    def startOp(self) -> None:
        self.watchdog.setEnable(False)

        if not self._engage():
            return

        try:
            self.req.setParameter("root/MachineControl/gotoJogMode", True).get()
            self.req.setParameter("root/MachineControl/gotoPauseMode", False).get()
            logging.info("JogMode / PauseMode 해제 완료.")
        except Exception as e:
            logging.warning(f"모드 해제 실패: {e}")

        # homemode / jumpmode 초기화
        try:
            self.req.setParameter(HOME_MODE_PATH, [0]).get()
            self.req.setParameter(JUMP_MODE_PATH, [0]).get()
            logging.info("homemode / jumpmode 초기화 완료.")
        except Exception as e:
            logging.warning(f"mode 초기화 실패: {e}")

        # actual_pos 구독 — 부모 경로 1회 구독, value[0~3] 인덱싱
        sub_pos = self.sub.subscribe([ACTUAL_PATH], 'pos_group', frq_divider=1)
        def _on_pos(msg):
            if msg and msg[0].value:
                for i in range(NUM_AXES):
                    self._actual_pos[i] = float(msg[0].value[i])
        sub_pos.notify(_on_pos)

        # actual_pos 1회 직접 폴링
        try:
            result = self.req.getParameter(ACTUAL_PATH).get()
            if result and result.value:
                for i in range(NUM_AXES):
                    self._actual_pos[i] = float(result.value[i])
        except Exception:
            pass
        self._hold_pos = list(self._actual_pos)
        logging.info(
            "초기 위치: "
            + ", ".join(f"축{i+1}={math.degrees(self._actual_pos[i]):+.2f}°" for i in range(NUM_AXES))
        )

        # homemode 구독
        sub_home = self.sub.subscribe([HOME_MODE_PATH], 'home_group', frq_divider=1)
        def _on_home(msg):
            if msg and msg[0].value and int(msg[0].value[0]) == 1:
                self._home_start_pos = list(self._actual_pos)
                self._home_requested = True
        sub_home.notify(_on_home)

        # jumpmode 구독
        sub_jump = self.sub.subscribe([JUMP_MODE_PATH], 'jump_group', frq_divider=1)
        def _on_jump(msg):
            if msg and msg[0].value and int(msg[0].value[0]) == 1:
                self._jump_requested = True
        sub_jump.notify(_on_jump)

        self.ready = True
        logging.info("4축 준비 완료 — jumpmode/homemode 대기 중")

    # ── 제어 루프 ─────────────────────────────────────────────────────────────
    def iterate(self) -> None:
        if not self.ready:
            time.sleep(0.1)
            return

        if self._home_requested:
            self._home_requested = False
            time.sleep(0.1)
            self.req.setParameter(HOME_MODE_PATH, [0]).get()
            self._home_requested = False
            self.move_to_home()
            return

        if self._jump_requested:
            self._jump_requested = False
            time.sleep(0.1)
            self.req.setParameter(JUMP_MODE_PATH, [0]).get()
            self._jump_requested = False
            logging.info("점프 궤적 실행")
            self._run_jump()
            logging.info("점프 완료 — 대기 중")
            return

        # 대기: _hold_pos 절대 위치 유지
        self._set_positions(self._hold_pos)
        time.sleep(CYCLE_TIME)

    # ── 종료 ──────────────────────────────────────────────────────────────────
    def onExit(self) -> None:
        logging.info("종료: 현재 위치 유지.")
        try:
            self._set_positions(list(self._actual_pos), blocking=True)
        except Exception as e:
            logging.warning(f"종료 시 리셋 실패: {e}")


if __name__ == "__main__":
    client_options = McxClientAppConfiguration(name="mcx-client-app", autoStart=False)
    client_options.set_config_paths(
        deployed_config="/etc/motorcortex/config/services/services_config.json",
        non_deployed_config="services_config.template.json"
    )
    client_options.load_config()

    print(f"\nUsing configuration: {client_options}\n\n")

    app = CspClientAppV5(client_options)

    _sigint_count = [0]
    def _sigint_handler(sig, frame):
        _sigint_count[0] += 1
        if _sigint_count[0] == 1:
            logging.info("Ctrl+C 감지 — 앱 종료 중... (한 번 더 누르면 강제 종료)")
            app.running.set(False)
        else:
            logging.info("강제 종료")
            os._exit(1)

    signal.signal(signal.SIGINT, _sigint_handler)

    app.run()

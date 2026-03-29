#
#   Developer : Coen Smeets (Coen@vectioneer.com)
#   All rights reserved. Copyright (c) 2025 VECTIONEER.
#

"""
CSP(Cyclic Synchronous Position) 모드 - SetpointGenerator 기반 직접 위치 제어

흐름:
  Python → SetpointGenerator01/setpoint
         → MachineControl/hostInJointPosition1[0]
         → MachineControl/jointPositionsTarget
         → actuatorControlLoop01 → 드라이브

사전 조건:
  - /etc/motorcortex/config/config.json 에서 SetpointGenerator Enable: true
  - /etc/motorcortex/config/linking/csp-control.link.json 존재
  - Motorcortex 재시작 완료
"""

import logging
import time
import math
logging.basicConfig(level=logging.INFO)

from src.mcx_client_app import McxClientApp, McxClientAppConfiguration

# ── 설정 상수 ──────────────────────────────────────────────────────────────────
AXIS_ID = 1

# CSP 위치 명령 경로 (SetpointGenerator → MachineControl/hostInJointPosition1)
CSP_SETPOINT_PATH = f"root/SetpointGenerators/SetpointGenerator{AXIS_ID:02d}/input"
CSP_GEN_BASE      = f"root/SetpointGenerators/SetpointGenerator{AXIS_ID:02d}"
CSP_ACTUAL_PATH   = "root/AxesControl/actuatorControlLoops/actuatorControlLoop01/motorPositionActual"

# 상태머신
STATE_CMD_PATH  = "root/Logic/stateCommand"
STATE_PATH      = "root/Logic/state"
ENGAGE_CMD      = 2       # GOTO_ENGAGED_E
ENGAGED_STATE   = 4       # ENGAGED_S
ENGAGE_TIMEOUT  = 10.0

# 궤적 파라미터 (원하는 궤적으로 교체)
AMPLITUDE_DEG   = 10.0
AMPLITUDE_RAD   = math.radians(AMPLITUDE_DEG)
FREQUENCY_HZ    = 0.1                        # 0.1 Hz = 10초 주기
CYCLE_TIME      = 0.004                      # 4 ms CSP 주기

# 엔코더
ENC_INC_PER_REV = 4096
ENC_TO_RAD      = (2.0 * math.pi) / ENC_INC_PER_REV
# ───────────────────────────────────────────────────────────────────────────────


class CspClientAppV2(McxClientApp):
    """
    SetpointGenerator를 통한 직접 위치 계산 CSP 제어 앱.
    iterate()에서 원하는 궤적을 계산해 SetpointGenerator에 직접 전송합니다.
    """

    def __init__(self, options: McxClientAppConfiguration):
        super().__init__(options)
        self.center_rad:  float = 0.0
        self.start_time:  float = 0.0
        self.csp_ready:   bool  = False

    # ── Engage 시퀀스 ──────────────────────────────────────────────────────────
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

    # ── 초기화 ────────────────────────────────────────────────────────────────
    def startOp(self) -> None:
        if not self._engage():
            return

        # Jog / Pause 모드 해제
        try:
            self.req.setParameter("root/MachineControl/gotoJogMode",   False).get()
            self.req.setParameter("root/MachineControl/gotoPauseMode", False).get()
            logging.info("JogMode / PauseMode 해제 완료.")
        except Exception as e:
            logging.warning(f"모드 해제 실패: {e}")

        # 현재 실제 위치를 중심값으로 읽어옴
        try:
            actual = self.req.getParameter(CSP_ACTUAL_PATH).get()
            if actual and actual.value:
                self.center_rad = float(actual.value[0]) * ENC_TO_RAD
                logging.info(f"초기 위치: {math.degrees(self.center_rad):.2f}°")
        except Exception as e:
            logging.warning(f"초기 위치 읽기 실패, 0으로 시작: {e}")
            self.center_rad = 0.0

        # SetpointGenerator PVA 리미터 설정 (0이면 움직임 불가)
        try:
            self.req.setParameter(CSP_GEN_BASE + "/maxVel",  [1.0]).get()   # rad/s
            self.req.setParameter(CSP_GEN_BASE + "/maxAcc",  [5.0]).get()   # rad/s²
            self.req.setParameter(CSP_GEN_BASE + "/maxJerk", [20.0]).get()  # rad/s³
            logging.info("SetpointGenerator PVA 리미터 설정 완료 (maxVel=1.0, maxAcc=5.0, maxJerk=20.0)")
        except Exception as e:
            logging.warning(f"PVA 리미터 설정 실패: {e}")

        # 초기 input 전송
        try:
            self.req.setParameter(CSP_SETPOINT_PATH, [self.center_rad]).get()
            self.start_time = time.time()
            self.csp_ready  = True
            logging.info(f"CSP 제어 시작 — SetpointGenerator{AXIS_ID:02d} 활성화")
            logging.info(f"궤적: ±{AMPLITUDE_DEG}°, {FREQUENCY_HZ} Hz")
        except Exception as e:
            logging.error(f"SetpointGenerator 초기화 실패: {e}")
            logging.error(f"경로 확인 필요: {CSP_SETPOINT_PATH}")

    # ── CSP 제어 루프 ─────────────────────────────────────────────────────────
    def iterate(self) -> None:
        if not self.csp_ready:
            time.sleep(0.1)
            return

        t = time.time() - self.start_time

        # ── 여기서 원하는 궤적을 계산하세요 ──────────────────────────────────
        # 예시: 중심 위치 기준 ±10° 사인파
        target_rad = self.center_rad + AMPLITUDE_RAD * math.sin(2.0 * math.pi * FREQUENCY_HZ * t)
        # ─────────────────────────────────────────────────────────────────────

        try:
            self.req.setParameter(CSP_SETPOINT_PATH, [target_rad]).get()
        except Exception as e:
            logging.warning(f"SetpointGenerator 쓰기 오류: {e}")

        time.sleep(CYCLE_TIME)

    # ── 종료 ──────────────────────────────────────────────────────────────────
    def onExit(self) -> None:
        logging.info("종료: 현재 위치 고정 후 정지.")
        try:
            self.req.setParameter(CSP_SETPOINT_PATH, [self.center_rad]).get()
        except Exception as e:
            logging.warning(f"종료 시 위치 설정 실패: {e}")
        try:
            self.req.setParameter(CSP_GEN_BASE + "/maxVel",  [0.0]).get()
            self.req.setParameter(CSP_GEN_BASE + "/maxAcc",  [0.0]).get()
            self.req.setParameter(CSP_GEN_BASE + "/maxJerk", [0.0]).get()
        except Exception:
            pass


if __name__ == "__main__":
    client_options = McxClientAppConfiguration(name="mcx-client-app")
    client_options.set_config_paths(
        deployed_config="/etc/motorcortex/config/services/services_config.json",
        non_deployed_config="services_config.template.json"
    )
    client_options.load_config()

    print(f"\nUsing configuration: {client_options}\n\n")

    app = CspClientAppV2(client_options)
    app.run()

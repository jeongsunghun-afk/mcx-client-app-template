#
#   Developer : Coen Smeets (Coen@vectioneer.com)
#   All rights reserved. Copyright (c) 2025 VECTIONEER.
#

"""
CSP(Cyclic Synchronous Position) 모드 - hostInJointAdditivePosition1 직접 제어

읽기 흐름 (현재 위치 파악):
  EtherCAT (0x6064)
        ↓  [master.xml 링크]
  motorPositionActual  (raw enc counts)  ← ACTUAL_PATH로 직접 읽음
        ↓  [ENC_TO_RAD: counts → rad]    ← Python에서 수동 변환
  현재 위치 (rad)

쓰기 흐름 (목표 위치 전송):
  Python → hostInJointAdditivePosition1  ← CSP_PATH에 additive offset(rad) 전송
        ↓  [Motorcortex 내부 연결]
  MachineControl/jointPositionsTarget
        ↓  [Motorcortex 내부 연결]
  actuatorControlLoop01
        ↓  [positionTransformation: rad → counts]
  motorPositionTarget  (raw enc counts)
        ↓  [master.xml 링크]
  EtherCAT (0x607A)

특징:
  - 현재 위치를 기준으로 오프셋(additive)을 매 주기 전송
  - SetpointGenerator 불필요, 초기 위치 읽기 불필요
  - 종료 시 0 전송 → 현재 위치 유지
"""

import logging
import time
import math
logging.basicConfig(level=logging.INFO)

from src.mcx_client_app import McxClientApp, McxClientAppConfiguration

# ── 설정 상수 ──────────────────────────────────────────────────────────────────
# [쓰기] additive offset(rad) → jointPositionsTarget → actuatorControlLoop → EtherCAT(0x607A)
CSP_PATH    = "root/MachineControl/hostInJointAdditivePosition1"
# [읽기] EtherCAT(0x6064) → motorPositionActual(raw enc counts) → Python에서 rad 변환
ACTUAL_PATH = "root/AxesControl/actuatorControlLoops/actuatorControlLoop01/motorPositionActual"

# 상태머신
STATE_CMD_PATH  = "root/Logic/stateCommand"
STATE_PATH      = "root/Logic/state"
ENGAGE_CMD      = 2       # GOTO_ENGAGED_E
ENGAGED_STATE   = 4       # ENGAGED_S
ENGAGE_TIMEOUT  = 10.0

# 엔코더 변환 (motorPositionActual: raw counts → rad, Motorcortex 내부 변환 미사용)
ENC_INC_PER_REV = 4096
ENC_TO_RAD      = (2.0 * math.pi) / ENC_INC_PER_REV

# 홈 복귀 파라미터
HOMING_THRESHOLD_RAD = math.radians(0.1)   # 0.1° 이내면 홈으로 간주
HOMING_VEL_DEG_S     = 30.0               # 홈 복귀 속도 (°/s)
HOMING_VEL_RAD_S     = math.radians(HOMING_VEL_DEG_S)

# 궤적 파라미터
AMPLITUDE_DEG = 10.0
AMPLITUDE_RAD = math.radians(AMPLITUDE_DEG)
FREQUENCY_HZ  = 0.1                        # 0.1 Hz = 10초 주기
CYCLE_TIME    = 0.001                      # 1 ms → 1 kHz CSP 주기
# ───────────────────────────────────────────────────────────────────────────────


class CspClientAppV3(McxClientApp):
    """
    hostInJointAdditivePosition1을 통한 직접 CSP 위치 제어 앱.
    iterate()에서 계산한 오프셋을 현재 위치에 더하는 방식으로 제어합니다.
    """

    def __init__(self, options: McxClientAppConfiguration):
        super().__init__(options)
        self.start_time:  float = 0.0
        self.home_offset: float = 0.0   # 홈 복귀 후 0점 기준 additive offset
        self.csp_ready:   bool  = False
        self._last_iter:  float = 0.0
        self._loop_count: int   = 0

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

    # ── 홈 복귀 시퀀스 ────────────────────────────────────────────────────────
    def _homing(self) -> bool:
        """현재 위치를 읽어 0이 아니면 선형 램프로 0으로 복귀."""
        try:
            # [읽기] EtherCAT(0x6064) → motorPositionActual(raw counts)
            actual = self.req.getParameter(ACTUAL_PATH).get()
            if not actual or not actual.value:
                logging.warning("위치 읽기 실패, 홈 생략.")
                return True
            # raw counts → rad (Python에서 직접 변환, Motorcortex positionTransformation 미사용)
            current_rad = float(actual.value[0]) * ENC_TO_RAD
        except Exception as e:
            logging.warning(f"위치 읽기 오류: {e}, 홈 생략.")
            return True

        logging.info(f"현재 위치: {math.degrees(current_rad):.2f}°")

        if abs(current_rad) <= HOMING_THRESHOLD_RAD:
            logging.info("홈 위치 이내, 홈 복귀 생략.")
            self.req.setParameter(CSP_PATH, [0.0]).get()
            self.home_offset = 0.0
            return True

        logging.info(f"홈 복귀 시작: {math.degrees(current_rad):.2f}° → 0° ({HOMING_VEL_DEG_S}°/s)")

        # additive offset을 -current_rad까지 램프 → 절대 위치 0으로 이동
        target_offset = -current_rad
        step = math.copysign(HOMING_VEL_RAD_S * CYCLE_TIME, target_offset)
        offset = 0.0

        while abs(target_offset - offset) > abs(step):
            offset += step
            try:
                self.req.setParameter(CSP_PATH, [offset]).get()
            except Exception as e:
                logging.warning(f"홈 쓰기 오류: {e}")
                return False
            time.sleep(CYCLE_TIME)

        self.req.setParameter(CSP_PATH, [target_offset]).get()
        self.home_offset = target_offset   # 0점 기준 오프셋 저장
        logging.info("홈 복귀 완료.")
        return True

    # ── 초기화 ────────────────────────────────────────────────────────────────
    def startOp(self) -> None:
        if not self._engage():
            return

        try:
            self.req.setParameter("root/MachineControl/gotoJogMode", True).get()
            self.req.setParameter("root/MachineControl/gotoPauseMode", False).get()
            logging.info("JogMode / PauseMode 해제 완료.")
        except Exception as e:
            logging.warning(f"모드 해제 실패: {e}")

        if not self._homing():
            logging.error("홈 복귀 실패, CSP 제어 중단.")
            return

        self.start_time = time.time()
        self.csp_ready  = True
        logging.info(f"CSP 제어 시작 — {CSP_PATH}")
        logging.info(f"궤적: ±{AMPLITUDE_DEG}°, {FREQUENCY_HZ} Hz")

    # ── CSP 제어 루프 ─────────────────────────────────────────────────────────
    def iterate(self) -> None:
        if not self.csp_ready:
            time.sleep(0.1)
            return

        now = time.perf_counter()
        self._loop_count += 1

        # 1초마다 실제 루프 주파수 출력
        if self._last_iter > 0.0:
            dt = now - self._last_iter
            if self._loop_count % 1000 == 0:
                logging.info(f"[루프] 실제 주기: {dt*1000:.2f} ms  ({1.0/dt:.0f} Hz)")
        self._last_iter = now

        t = now - self.start_time

        # ── 여기서 원하는 궤적을 계산하세요 ──────────────────────────────────
        # 0점 기준 ±AMPLITUDE_DEG 사인파 (home_offset이 0점 유지)
        offset_rad = self.home_offset + AMPLITUDE_RAD * math.sin(2.0 * math.pi * FREQUENCY_HZ * t)
        # ─────────────────────────────────────────────────────────────────────

        # [쓰기] offset_rad → hostInJointAdditivePosition1 → jointPositionsTarget
        #         → actuatorControlLoop → motorPositionTarget(counts) → EtherCAT(0x607A)
        self.req.setParameter(CSP_PATH, [offset_rad])

        time.sleep(CYCLE_TIME)

    # ── 종료 ──────────────────────────────────────────────────────────────────
    def onExit(self) -> None:
        logging.info("종료: 오프셋 0으로 리셋.")
        try:
            self.req.setParameter(CSP_PATH, [0.0]).get()
        except Exception as e:
            logging.warning(f"종료 시 오프셋 리셋 실패: {e}")


if __name__ == "__main__":
    client_options = McxClientAppConfiguration(name="mcx-client-app")
    client_options.set_config_paths(
        deployed_config="/etc/motorcortex/config/services/services_config.json",
        non_deployed_config="services_config.template.json"
    )
    client_options.load_config()

    print(f"\nUsing configuration: {client_options}\n\n")

    app = CspClientAppV3(client_options)
    app.run()

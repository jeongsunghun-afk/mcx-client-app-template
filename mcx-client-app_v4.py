#
#   Developer : Coen Smeets (Coen@vectioneer.com)
#   All rights reserved. Copyright (c) 2025 VECTIONEER.
#

"""
CSP(Cyclic Synchronous Position) 모드 - hostInJointAdditivePosition1 4축 동시 제어

읽기 흐름 (현재 위치 파악, 4축):
  EtherCAT (0x6064) × 4
        ↓  [master.xml 링크]
  motorPositionActual × 4 (raw enc counts)  ← ACTUAL_PATHS[0~3] 으로 직접 읽음
        ↓  [ENC_TO_RAD: counts → rad]        ← Python에서 수동 변환
  현재 위치 4축 (rad)

쓰기 흐름 (목표 위치 전송, 4축 동시):
  Python → hostInJointAdditivePosition1  ← CSP_PATH에 [r0, r1, r2, r3] (4-element list) 전송
        ↓  [Motorcortex 내부 연결]
  MachineControl/jointPositionsTarget (4축)
        ↓  [Motorcortex 내부 연결]
  actuatorControlLoop01 ~ 04
        ↓  [positionTransformation: rad → counts]
  motorPositionTarget × 4 (raw enc counts)
        ↓  [master.xml 링크]
  EtherCAT (0x607A) × 4

특징:
  - 4축을 하나의 setParameter([r0,r1,r2,r3]) 호출로 동시 제어
  - 각 축의 home_offset 독립 관리 → 홈 복귀 후 각 축 기준 ±10° 유지
  - 홈 복귀: 4축 클로즈드루프 동시 수행 (모든 축이 threshold 이내가 될 때까지)
  - 종료 시 [0,0,0,0] 전송 → 현재 위치 유지
"""

import logging
import time
import math
logging.basicConfig(level=logging.INFO)

from src.mcx_client_app import McxClientApp, McxClientAppConfiguration

# ── 설정 상수 ──────────────────────────────────────────────────────────────────
NUM_AXES = 4

# 실제 모터 / 시뮬레이션 축 구분 (True = 실제 EtherCAT 연결, False = 시뮬레이션)
# 축1: 실제 모터,  축2~4: 시뮬레이션 (드라이브 미연결)
AXIS_IS_REAL = [True, False, False, False]

# [드라이브 비활성화] 시뮬레이션 축의 드라이브 출력 차단
# setParameter(DISABLE_DRIVE_PATH, [False, True, True, True])
# → 축1 활성, 축2~4 비활성
DISABLE_DRIVE_PATH = "root/DriveLogic/disableDrive"

# [쓰기] 4-element additive offset list(rad) → jointPositionsTarget → actuatorControlLoop01~04 → EtherCAT(0x607A)
CSP_PATH     = "root/MachineControl/hostInJointAdditivePosition1"

# [읽기] EtherCAT(0x6064) × 4 → motorPositionActual(raw enc counts) → Python에서 rad 변환
# 시뮬레이션 축(AXIS_IS_REAL=False)은 homing 시 읽기 생략, home_offset=0 고정
ACTUAL_PATHS = [
    f"root/AxesControl/actuatorControlLoops/actuatorControlLoop0{i}/motorPositionActual"
    for i in range(1, NUM_AXES + 1)
]

# 상태머신
STATE_CMD_PATH = "root/Logic/stateCommand"
STATE_PATH     = "root/Logic/state"
ENGAGE_CMD     = 2       # GOTO_ENGAGED_E
ENGAGED_STATE  = 4       # ENGAGED_S
ENGAGE_TIMEOUT = 10.0

# 엔코더 변환 (motorPositionActual: raw counts → rad, Motorcortex 내부 변환 미사용)
ENC_INC_PER_REV = 4096
ENC_TO_RAD      = (2.0 * math.pi) / ENC_INC_PER_REV

# 홈 복귀 파라미터
HOMING_THRESHOLD_RAD = math.radians(0.1)   # 0.1° 이내면 홈으로 간주
HOMING_VEL_DEG_S     = 30.0               # 홈 복귀 속도 (°/s)
HOMING_VEL_RAD_S     = math.radians(HOMING_VEL_DEG_S)

# 궤적 파라미터 (4축 공통)
AMPLITUDE_DEG = 10.0
AMPLITUDE_RAD = math.radians(AMPLITUDE_DEG)
FREQUENCY_HZ  = 0.1                        # 0.1 Hz = 10초 주기
CYCLE_TIME    = 0.001                      # 1 ms → 1 kHz CSP 주기
# ───────────────────────────────────────────────────────────────────────────────


class CspClientAppV4(McxClientApp):
    """
    hostInJointAdditivePosition1을 통한 4축 동시 CSP 위치 제어 앱.
    iterate()에서 [r0, r1, r2, r3] 를 한 번의 setParameter 호출로 전송합니다.
    """

    def __init__(self, options: McxClientAppConfiguration):
        super().__init__(options)
        self.start_time:   float      = 0.0
        self.home_offsets: list[float] = [0.0] * NUM_AXES  # 4축 각각의 home additive offset
        self.csp_ready:    bool        = False
        self._last_iter:   float       = 0.0
        self._loop_count:  int         = 0

    # ── 단일 축 actual 읽기 헬퍼 ───────────────────────────────────────────────
    def _read_actual(self, axis: int) -> float | None:
        """axis(0-based) 의 motorPositionActual(raw counts) → rad 반환. 실패 시 None."""
        try:
            # [읽기] EtherCAT(0x6064) → motorPositionActual(raw counts)
            reply = self.req.getParameter(ACTUAL_PATHS[axis]).get()
            if not reply or not reply.value:
                return None
            # raw counts → rad (Python에서 직접 변환)
            return float(reply.value[0]) * ENC_TO_RAD
        except Exception:
            return None

    # ── Engage 시퀀스 (v3 _engage() 동일) ──────────────────────────────────────
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

    # ── 홈 복귀 시퀀스 (4축 동시 클로즈드루프) ────────────────────────────────
    def _homing(self) -> bool:
        """4축 actual을 매 스텝 읽어 모든 축이 0° threshold 이내가 될 때까지 동시 조정."""

        # 축별 초기 위치 읽기 (시뮬레이션 축은 0.0으로 고정)
        currents: list[float] = []
        for i in range(NUM_AXES):
            if not AXIS_IS_REAL[i]:
                # 시뮬레이션 축: 실제 EtherCAT 없음 → 읽기 생략, 0°로 간주
                currents.append(0.0)
                logging.info(f"축{i+1} [시뮬] Homing 생략, home_offset=0°")
                continue
            val = self._read_actual(i)
            if val is None:
                logging.warning(f"축{i+1} 위치 읽기 실패, Homing 생략.")
                return True
            currents.append(val)
            logging.info(f"축{i+1} [실제] 현재 위치: {math.degrees(val):.2f}°")

        # 실제 축 중 홈 이내인지 확인
        real_done = all(
            abs(currents[i]) <= HOMING_THRESHOLD_RAD
            for i in range(NUM_AXES) if AXIS_IS_REAL[i]
        )
        if real_done:
            logging.info("실제 축 전체 홈 위치 이내, Homing 생략.")
            self.req.setParameter(CSP_PATH, [0.0] * NUM_AXES).get()
            self.home_offsets = [0.0] * NUM_AXES
            return True

        logging.info(f"실제 축 Homing 시작 ({HOMING_VEL_DEG_S}°/s), 시뮬 축은 0 유지")

        # [클로즈드루프] 실제 축만 homing, 시뮬 축은 offset=0 고정
        offsets: list[float] = [0.0] * NUM_AXES
        while True:
            # 종료 조건: 실제 축 전부 threshold 이내
            if all(
                abs(currents[i]) <= HOMING_THRESHOLD_RAD
                for i in range(NUM_AXES) if AXIS_IS_REAL[i]
            ):
                break

            for i in range(NUM_AXES):
                if not AXIS_IS_REAL[i]:
                    continue   # 시뮬 축: offset 변경 없음
                if abs(currents[i]) > HOMING_THRESHOLD_RAD:
                    step = math.copysign(HOMING_VEL_RAD_S * CYCLE_TIME, -currents[i])
                    offsets[i] += step

            # [쓰기] 4축 동시 전송 (시뮬 축 offset=0 포함)
            try:
                self.req.setParameter(CSP_PATH, offsets).get()
            except Exception as e:
                logging.warning(f"Homing 쓰기 오류: {e}")
                return False

            time.sleep(CYCLE_TIME)

            # 실제 축만 actual 갱신 (클로즈드루프)
            for i in range(NUM_AXES):
                if not AXIS_IS_REAL[i]:
                    continue
                val = self._read_actual(i)
                if val is not None:
                    currents[i] = val

        # 최종 offset 확정 전송
        self.req.setParameter(CSP_PATH, offsets).get()
        # homing이 각 축 motorPositionActual=0°를 달성한 additive 값을 기준으로 저장
        # CSP: home_offsets[i] + AMPLITUDE*sin(t) → 각 축 0° 기준 ±10° 유지
        self.home_offsets = list(offsets)
        logging.info(
            "4축 Homing 완료. home_offsets: "
            + ", ".join(f"축{i+1}={math.degrees(v):+.2f}°" for i, v in enumerate(offsets))
        )
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

        # ── 시뮬레이션 축 드라이브 비활성화 ────────────────────────────────────
        # AXIS_IS_REAL 기반: 실제 축=False(활성), 시뮬 축=True(비활성)
        disable_mask = [not real for real in AXIS_IS_REAL]
        sim_axes = [i+1 for i, real in enumerate(AXIS_IS_REAL) if not real]
        try:
            self.req.setParameter(DISABLE_DRIVE_PATH, disable_mask).get()
            logging.info(f"드라이브 비활성화 완료 — 시뮬레이션 축: {sim_axes}")
        except Exception as e:
            logging.warning(f"드라이브 비활성화 실패: {e}")

        if not self._homing():
            logging.error("Homing 실패, CSP 제어 중단.")
            return

        self.start_time = time.time()
        self.csp_ready  = True
        logging.info(f"4축 CSP 제어 시작 — {CSP_PATH}")
        logging.info(f"궤적: ±{AMPLITUDE_DEG}° × {NUM_AXES}축, {FREQUENCY_HZ} Hz")

    # ── CSP 제어 루프 (4축 동시) ───────────────────────────────────────────────
    def iterate(self) -> None:
        if not self.csp_ready:
            time.sleep(0.1)
            return

        now = time.perf_counter()
        self._loop_count += 1

        # 1초마다 실제 루프 주파수 출력 (v3 참조, 시간 기반)
        if self._last_iter > 0.0:
            dt = now - self._last_iter
            if self._loop_count % 1000 == 0:
                logging.info(f"[루프] 실제 주기: {dt*1000:.2f} ms  ({1.0/dt:.0f} Hz)")
        self._last_iter = now

        t = now - self.start_time

        # ── 4축 궤적 계산 ──────────────────────────────────────────────────────
        # home_offsets[i] 기준 ±AMPLITUDE 사인파 (축별 독립 home 유지)
        # ※ 여기서 원하는 궤적으로 교체하세요 (axes별 다른 amplitude/phase 등)
        sin_val = AMPLITUDE_RAD * math.sin(2.0 * math.pi * FREQUENCY_HZ * t)
        offsets = [self.home_offsets[i] + sin_val for i in range(NUM_AXES)]
        # ───────────────────────────────────────────────────────────────────────

        # [쓰기] [r0, r1, r2, r3] → hostInJointAdditivePosition1 (4축 동시, 단일 호출)
        #         → jointPositionsTarget → actuatorControlLoop01~04 → EtherCAT(0x607A)
        self.req.setParameter(CSP_PATH, offsets)

        time.sleep(CYCLE_TIME)

    # ── 종료 ──────────────────────────────────────────────────────────────────
    def onExit(self) -> None:
        logging.info("종료: 4축 오프셋 0으로 리셋.")
        try:
            # [쓰기] 4축 동시 0 → 현재 위치 유지
            self.req.setParameter(CSP_PATH, [0.0] * NUM_AXES).get()
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

    app = CspClientAppV4(client_options)
    app.run()

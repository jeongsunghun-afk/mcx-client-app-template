#
#   Developer : Coen Smeets (Coen@vectioneer.com)
#   All rights reserved. Copyright (c) 2025 VECTIONEER.
#

"""
Signal Generator를 이용한 위치 제어 클라이언트 앱.

Motorcortex 내장 Signal Generator를 사용하여 사인파 위치 명령을 생성합니다.
"""

import logging
import time
import math
logging.basicConfig(level=logging.INFO)

from src.mcx_client_app import McxClientApp, McxClientAppConfiguration

# ── 설정 상수 ──────────────────────────────────────────────
STATE_COMMAND   = "root/Logic/stateCommand"
STATE_PARAM     = "root/Logic/state"
ENGAGE_CMD      = 2                  # GOTO_ENGAGED_E
ENGAGED_STATE   = 4                  # ENGAGED_S
ENGAGE_TIMEOUT  = 10.0               # 초

AXIS_ID = 1                          # 제어할 축 번호
SIG_GEN_BASE  = f"root/SignalGenerators/SignalGenerator{AXIS_ID:02d}"
SIG_GEN_ENABLE = SIG_GEN_BASE + "/enable"

AMPLITUDE_DEG = 10.0                 # 진폭 (도)
AMPLITUDE_RAD = math.radians(AMPLITUDE_DEG)  # → 라디안
FREQUENCY_HZ  = 0.1                  # 주파수 (Hz)
SIGNAL_TYPE_SINE = 4                 # 사인파
# ──────────────────────────────────────────────────────────


class CspClientApp(McxClientApp):

    def _engage(self) -> bool:
        """Engage 시퀀스: 상태가 이미 ENGAGED면 스킵."""
        try:
            state = self.req.getParameter(STATE_PARAM).get()
            if state and state.value and state.value[0] == ENGAGED_STATE:
                logging.info("이미 Engaged 상태입니다.")
                return True
        except Exception:
            pass

        logging.info("Engage 명령 전송 중...")
        self.req.setParameter(STATE_COMMAND, [ENGAGE_CMD]).get()

        deadline = time.time() + ENGAGE_TIMEOUT
        while time.time() < deadline:
            try:
                state = self.req.getParameter(STATE_PARAM).get()
                if state and state.value and state.value[0] == ENGAGED_STATE:
                    logging.info("Engage 완료.")
                    return True
            except Exception:
                pass
            time.sleep(0.1)

        logging.error("Engage 타임아웃!")
        return False

    def startOp(self) -> None:
        if not self._engage():
            return

        logging.info("모드 초기화 중...")
        try:
            self.req.setParameter("root/MachineControl/gotoJogMode",   False).get()
            self.req.setParameter("root/MachineControl/gotoPauseMode", False).get()
            logging.info("JogMode / PauseMode 해제 완료")
        except Exception as e:
            logging.warning(f"모드 해제 실패: {e}")

        logging.info("Signal Generator 설정 중...")
        try:
            self.req.setParameter(SIG_GEN_BASE + "/signalType", SIGNAL_TYPE_SINE).get()
            self.req.setParameter(SIG_GEN_BASE + "/amplitude",  AMPLITUDE_RAD).get()
            self.req.setParameter(SIG_GEN_BASE + "/frequency",  FREQUENCY_HZ * 2 * math.pi).get()
            self.req.setParameter(SIG_GEN_ENABLE, True).get()
            logging.info(f"Signal Generator 활성화: ±{AMPLITUDE_DEG}도, {FREQUENCY_HZ}Hz")
        except Exception as e:
            logging.warning(f"Signal Generator 설정 실패: {e}")

    def iterate(self) -> None:
        time.sleep(0.1)

    def onExit(self) -> None:
        logging.info("Signal Generator 비활성화 중...")
        try:
            self.req.setParameter(SIG_GEN_ENABLE, False).get()
            logging.info("Signal Generator 정지 완료")
        except Exception as e:
            logging.warning(f"Signal Generator 정지 실패: {e}")


if __name__ == "__main__":
    client_options = McxClientAppConfiguration(name="mcx-client-app")
    client_options.set_config_paths(
        deployed_config="/etc/motorcortex/config/services/services_config.json",
        non_deployed_config="services_config.template.json"
    )
    client_options.load_config()

    print(f"\nUsing configuration: {client_options}\n\n")

    app = CspClientApp(client_options)
    app.run()

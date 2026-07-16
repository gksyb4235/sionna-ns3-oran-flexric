"""
kyunghee_server.py
─────────────────────────────────────────────────────────────────────
ns3sionna(TU Berlin)의 SionnaEnv를 상속해서, 경희대 캠퍼스 씬을 쓰는 ns-3 +
FlexRIC 파이프라인에 필요한 것만 얹는다:

  - 저장소 내부 GUI(contrib/sionna/gui) 실시간 위치/RSRP 릴레이
  - gNB(ConstantMobility)/UE(RandomWalkMobility 또는 ExternalMobility) 구분
  - ns-3 RRC identity와 SUMO lifecycle 상태 릴레이

ray tracing·모빌리티·캐싱 자체는 SionnaEnv(부모 클래스)가 담당하고,
이 파일은 그 결과를 GUI로 흘려보내는 얇은 래퍼다. 로컬 SionnaEnv fork에는
Sionna RT 2.x/Dr.Jit GPU 감지와 Python 3.12 호환성 수정이 적용돼 있다.
"""

import argparse
import os
import sys

import zmq

from ns3sionna_server import SionnaEnv
from mobility import ConstantMobility, ExternalMobility, RandomWalkMobility


DEFAULT_KYUNGHEE_SCENE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "models", "kyunghee"
)

class KyungheeSionnaEnv(SionnaEnv):
    def __init__(self, *args, tx_power_dbm: float = 43.0,
                 gui_src: str | None = None, gui_host: str = "localhost",
                 gui_max_gnbs: int = 0, **kwargs):
        super().__init__(*args, **kwargs)
        self.tx_power_dbm = tx_power_dbm
        self.gui_max_gnbs = gui_max_gnbs
        self._gui_name_by_node = {}

        # {rx_node_id: {tx_node_id: rsrp_dbm}} — 지금까지 관측된 것만 채워짐
        # (P2P/P2MP 요청 패턴에 따라 점진적으로 채워짐, 매 프레임 전체가 아님)
        self._rsrp_by_ue = {}
        # ns-3가 ChannelStateRequest에 실어 보낸 실제 RRC 상태.
        # RT 최대 수신전력 셀과 혼동하지 않고 별도 보존한다.
        self._radio_state_by_ue = {}

        self.gui_client = None
        if gui_src:
            sys.path.insert(0, gui_src)
            try:
                from sionna_rt_gui.zmq_bridge import ZMQBridgeClient
            except ImportError as exc:
                print(f"[kyunghee_server] GUI relay disabled: {exc}")
            else:
                self.gui_client = ZMQBridgeClient(host=gui_host)
                self.gui_client.connect()
                self._harden_gui_sockets()
                print("[kyunghee_server] GUI relay connected")

    def init_simulation_env(self, sim_init_msg):
        """Initialize RT and assign stable GUI names independent of ns-3 node IDs.

        EN-DC creates the mmWave nodes before its LTE anchor. ``--gui-max-gnbs``
        therefore exposes only the requested leading constant nodes and hides the
        extra LTE anchor from the Sionna 3D panel.
        """
        success, message = super().init_simulation_env(sim_init_msg)
        if not success:
            return success, message

        gnb_ids = [node_id for node_id, mobility in self.node_info.items()
                   if isinstance(mobility, ConstantMobility)]
        ue_ids = [node_id for node_id, mobility in self.node_info.items()
                  if isinstance(mobility, (ExternalMobility, RandomWalkMobility))]
        if self.gui_max_gnbs > 0:
            gnb_ids = gnb_ids[:self.gui_max_gnbs]

        self._gui_name_by_node = {
            **{node_id: f"gnb{i + 1}" for i, node_id in enumerate(gnb_ids)},
            **{node_id: f"ue{i + 1}" for i, node_id in enumerate(ue_ids)},
        }
        if self.gui_client is not None:
            print(f"[kyunghee_server] GUI node map: {self._gui_name_by_node}")
        return success, message

    def _harden_gui_sockets(self):
        """LINGER=0을 강제한다.

        기본 LINGER(-1)면, GUI가 안 떠 있어 응답을 못 받은 미전송 메시지가
        소켓에 남아있는 상태에서 disconnect()->ctx.term()이 그 메시지를
        전달하려고 무한 대기해버린다 (재연결 자체가 통째로 멈춤).
        """
        if self.gui_client is None:
            return
        if self.gui_client._ue_sock:
            self.gui_client._ue_sock.setsockopt(zmq.LINGER, 0)
        if self.gui_client._kpi_sock:
            self.gui_client._kpi_sock.setsockopt(zmq.LINGER, 0)

    def _safe_gui_call(self, fn, *args):
        """GUI가 안 떠 있거나 잠깐 느려도 시뮬레이션 자체는 절대 죽지 않게 감싼다.

        ZMQBridgeClient의 REQ 소켓은 recv가 한 번 타임아웃되면(RCVTIMEO=1000ms)
        상태 머신이 깨져서 이후 모든 send()가 즉시 ZMQError를 던진다 — 이 경우
        소켓을 통째로 재연결해서 GUI가 다시 뜨면 자동으로 복구되게 한다.
        """
        if self.gui_client is None:
            return
        try:
            result = fn(*args)
            if result is False:
                raise TimeoutError("GUI relay request timed out")
        except Exception as e:  # noqa: BLE001 — GUI 릴레이는 best-effort
            print(f"[kyunghee_server] GUI relay call failed ({e}); reconnecting")
            try:
                self.gui_client.disconnect()
            except Exception:
                pass
            self.gui_client.connect()
            self._harden_gui_sockets()

    def close_gui(self):
        if self.gui_client is not None:
            self.gui_client.disconnect()
            self.gui_client = None

    def _role_of(self, node_id: int) -> str:
        info = self.node_info.get(node_id)
        if isinstance(info, ExternalMobility):
            return "ue"
        if isinstance(info, ConstantMobility):
            return "gnb"
        if isinstance(info, RandomWalkMobility):
            return "ue"
        return "unknown"

    def _gui_name(self, node_id: int) -> str | None:
        return self._gui_name_by_node.get(node_id)

    def _capture_radio_state(self, csi_req):
        if not csi_req.HasField("ue_radio_state"):
            return
        state = csi_req.ue_radio_state
        current = {
            "ue_node_id": state.node_id,
            "imsi": state.imsi,
            "rnti": state.rnti,
            "serving_cell_id": state.serving_cell_id,
            "serving_gnb_node_id": state.serving_gnb_node_id,
            "active": state.active,
        }
        previous = self._radio_state_by_ue.get(state.node_id)
        self._radio_state_by_ue[state.node_id] = current
        if self.VERBOSE and current != previous:
            print(f"[kyunghee_server] ns-3 RRC state: {current}")

    def _relay_response(self, chan_response):
        """reply_wrapper.channel_state_response(이미 채워진 것)를 GUI로 릴레이."""
        if self.gui_client is None:
            return
        for csi in chan_response.csi:
            tx_id = csi.tx_node.id
            tx_pos = csi.tx_node.position
            tx_role = self._role_of(tx_id)
            tx_name = self._gui_name(tx_id)

            if tx_name is not None:
                send_tx = (self.gui_client.send_gnb_position if tx_role == "gnb"
                           else self.gui_client.send_ue_position)
                self._safe_gui_call(send_tx, tx_name, [tx_pos.x, tx_pos.y, tx_pos.z])

            for rx in csi.rx_nodes:
                rx_id = rx.id
                rx_role = self._role_of(rx_id)
                rx_name = self._gui_name(rx_id)
                rx_pos = rx.position

                if rx_name is not None:
                    send_rx = (self.gui_client.send_gnb_position if rx_role == "gnb"
                               else self.gui_client.send_ue_position)
                    self._safe_gui_call(send_rx, rx_name, [rx_pos.x, rx_pos.y, rx_pos.z])

                # wb_loss는 이미 rsrp.py와 동일한 비코히런트 광대역 평균전력
                # 기준(-10log10(mean(|h|^2)))으로 계산돼 있음 — 그대로 dBm 변환.
                rsrp_dbm = self.tx_power_dbm - rx.wb_loss

                gnb_id, ue_id = (tx_id, rx_id) if tx_role == "gnb" else (rx_id, tx_id)
                if (tx_role != rx_role and self._gui_name(gnb_id) is not None
                        and self._gui_name(ue_id) is not None):
                    self._rsrp_by_ue.setdefault(ue_id, {})[gnb_id] = rsrp_dbm

        # RSRP 최댓값은 물리 채널의 best-cell 후보일 뿐이다. serving_cell은
        # ChannelStateRequest에 동봉된 ns-3 RRC/target-eNB 상태만 사용한다.
        ue_stats = []
        for ue_id, by_gnb in self._rsrp_by_ue.items():
            best_gnb_id, _ = max(by_gnb.items(), key=lambda kv: kv[1])
            radio = self._radio_state_by_ue.get(ue_id, {})
            serving_gnb_node_id = radio.get("serving_gnb_node_id", 0)
            serving_name = self._gui_name(serving_gnb_node_id)
            stat = {
                "ue": self._gui_name(ue_id),
                "ue_node_id": ue_id,
                "imsi": radio.get("imsi", 0),
                "rnti": radio.get("rnti", 0),
                "rsrp_dbm": {self._gui_name(g): v for g, v in by_gnb.items()},
                "serving_cell": serving_name,
                "serving_cell_id": radio.get("serving_cell_id", 0),
                "best_rsrp_cell": self._gui_name(best_gnb_id),
                "active": radio.get("active", True),
            }
            ue_stats.append(stat)
        if ue_stats:
            self._safe_gui_call(self.gui_client.send_kpi, ue_stats)

    def compute_cfr_classic(self, csi_req, reply_wrapper, req_mode):
        self._capture_radio_state(csi_req)
        num_links = super().compute_cfr_classic(csi_req, reply_wrapper, req_mode)
        self._relay_response(reply_wrapper.channel_state_response)
        return num_links

    def compute_cfr_with_lookahead(self, csi_req, reply_wrapper):
        self._capture_radio_state(csi_req)
        num_links = super().compute_cfr_with_lookahead(csi_req, reply_wrapper)
        self._relay_response(reply_wrapper.channel_state_response)
        return num_links


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_folder", type=str,
                        default=DEFAULT_KYUNGHEE_SCENE_DIR,
                        help="Kyunghee.xml이 있는 씬 디렉토리 (기본값: 저장소 내부 models/kyunghee)")
    parser.add_argument("--single_run", action="store_true")
    parser.add_argument("--port", type=int, default=5555,
                        help="ns-3 ZMQ REP port (default: 5555)")
    parser.add_argument("--default_mode", type=int, default=SionnaEnv.MODE_P2MP)
    parser.add_argument("--rt_fast", action="store_true")
    parser.add_argument("--rt_max_parallel_links", type=int, default=256)
    parser.add_argument("--est_csi", type=bool, default=True)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--tx_power_dbm", type=float, default=43.0,
                        help="gNB 총 송신전력(dBm) — RSRP = tx_power_dbm - wb_loss")
    parser.add_argument("--gui-src", default=os.environ.get("SIONNA_GUI_SRC"),
                        help="contrib/sionna/gui/src 경로; 생략하면 GUI relay 비활성")
    parser.add_argument("--gui-host", default="localhost",
                        help="Polyscope GUI ZMQ bridge host")
    parser.add_argument("--gui-max-gnbs", type=int, default=0,
                        help="GUI에 노출할 선행 ConstantMobility gNB 수; 0은 전체")
    args = parser.parse_args()

    print("kyunghee_server v1.0 (based on ns3sionna)")
    while True:
        print(f"Using config: model_folder={args.model_folder}, mode={args.default_mode}, "
              f"rt_fast={args.rt_fast}, tx_power_dbm={args.tx_power_dbm}")
        print("Waiting for new job ...")
        env = KyungheeSionnaEnv(args.model_folder, args.rt_fast, args.default_mode,
                                args.rt_max_parallel_links, args.est_csi,
                                VERBOSE=args.verbose, tx_power_dbm=args.tx_power_dbm,
                                gui_src=args.gui_src, gui_host=args.gui_host,
                                gui_max_gnbs=args.gui_max_gnbs)
        try:
            env.run(f"tcp://*:{args.port}")
        finally:
            env.close_gui()

        if args.single_run:
            break

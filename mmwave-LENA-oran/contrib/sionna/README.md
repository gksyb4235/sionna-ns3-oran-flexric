# ns3sionna integration for ns-O-RAN-flexric

This directory vendors the `tkn-tub/ns3sionna` ns-3 module and its Python
server so a checkout of this repository contains both ends of the ZMQ/
Protocol Buffers bridge.

- Upstream: <https://github.com/tkn-tub/ns3sionna>
- Imported revision: `c896b858cd9d14a1fd36a4c8116f1538b8ebbcdf`
- Local integration: the C++ module is adapted to this ns-3/mmWave fork and
  `model/ns3sionna/kyunghee_server.py` loads the Kyunghee Sionna RT scene.
- Self-contained scene: `model/ns3sionna/models/kyunghee/Kyunghee.xml` and its
  `meshes/` directory are real files in this repository, with no dependency on
  `sionna-oai-flexric-smo` or `Sionna_Tutorial` paths.
- Current radio-channel scope: SISO. The upstream experimental phased-array
  model is not compiled against this fork.

The generated Protocol Buffers files (`model/message.pb.{cc,h}` and
`model/ns3sionna/common/message_pb2.py`) are intentionally not versioned.
They are regenerated from `model/message.proto` by CMake during ns-3
configuration/build.

Runtime and build instructions are in the repository-level `README.md`.
The Python implementation under `model/ns3sionna` retains its upstream MIT
license in `model/ns3sionna/LICENSE`; ns-3 C++ sources retain their individual
license headers.

The supported Python runtime is the repository-local
`/home/user/ns-O-RAN-flexric/.venv`, installed from the root
`requirements.txt`. It contains standalone Sionna RT/Mitsuba/Dr.Jit and does
not require TensorFlow. Full Sionna PHY/SYS is an optional layer installed from
`requirements-sys.txt` for offline link/system-level research.

## SUMO/external UE mobility

`scratch/scenario-zero-sionna-kyunghee.cc` accepts a CSV exported from SUMO:

```text
time_s,ue_id,x,y,z,active
0.0,person0,30.0,20.0,1.5,true
0.3,person0,26.0,20.0,1.5,false
0.5,person0,22.0,18.0,1.5,true
```

Run it with `--sumoTrace=contrib/sionna/examples/sumo-ue-trace.csv`. Distinct IDs
are mapped to the pre-created ns-3 UE pool in first-seen order; therefore
`N_Ues` must equal the number of distinct IDs. Position changes are applied in
ns-3 and synchronized to Sionna on channel requests. `active=false` stops that
UE's offered downlink traffic, and a later `active=true` starts a new traffic
session while reusing its NetDevice/RRC context.

This is intentionally a logical lifecycle, not runtime deletion/recreation of
an ns-3 Node. It is reliable for load/congestion experiments and repeated
appearance, but the inactive UE remains RRC-attached. Exact detach/reattach and
KPM connected-UE-count semantics require a separate RRC lifecycle extension.

## Repository-local Kyunghee 3D/RadioMap GUI demo

The Polyscope GUI is vendored in `contrib/sionna/gui`; it no longer imports the
old `sionna-oai-flexric-smo` source tree or its scene. Both the GUI and the
channel server load the single canonical scene under
`model/ns3sionna/models/kyunghee`.

The ready-made topology uses two mmWave gNBs and three UEs:

```text
gnb1 = [-150, -267, 50]       gnb2 = [-159, 105, 30]
ue1: [-155,-100,1.5] -> [-155,-60,1.5] -> [-155,-20,1.5]
ue2: [23,-174,1.5]  -> [-20,-174,1.5] -> [-60,-174,1.5]
ue3: [-47,-3,1.5]   -> [-80,-80,1.5]  -> [-110,-160,1.5]
```

The trajectory CSV interpolates these waypoints at 0.01 simulated-second
intervals so the short RT demo visibly moves. Replace it with the same SUMO CSV
schema for a real time axis.

Start the components in separate terminals, in this order.

### 1. Polyscope 3D GUI

```bash
cd /home/user/ns-O-RAN-flexric
source .venv/bin/activate
python -u mmwave-LENA-oran/contrib/sionna/gui/scripts/run_kyunghee_demo.py
```

Add `--compute-radio-map` to calculate the initial RadioMap before the window is
shown. Without it, the scene, named gNB/UE markers and planned trajectory lines
open immediately; RadioMap can be computed from the GUI panel. The GUI listens
on TCP 5600 for positions and TCP 5601 for KPI data. Per-position path solving
is disabled in this process because the ns-3 Sionna server is the authoritative
channel solver.

### 2. Sionna RT channel server and GUI relay

```bash
cd /home/user/ns-O-RAN-flexric
source .venv/bin/activate
cd mmwave-LENA-oran/contrib/sionna/model/ns3sionna
python -u kyunghee_server.py \
  --port 5556 \
  --single_run \
  --rt_fast \
  --gui-src /home/user/ns-O-RAN-flexric/mmwave-LENA-oran/contrib/sionna/gui/src \
  --gui-max-gnbs 2
```

The stable GUI mapping is `gnb1`, `gnb2`, `ue1`, `ue2`, `ue3`; the extra LTE
anchor required by EN-DC is not rendered. A link for which the fast RT solver
finds no ray is returned to ns-3 as 300 dB loss with geometric propagation
delay instead of terminating the server.

### 3. FlexRIC and monitoring xApp (optional E2 path)

```bash
cd /home/user/ns-O-RAN-flexric/flexric/build/examples/ric
./nearRT-RIC
```

After the scenario starts, run:

```bash
cd /home/user/ns-O-RAN-flexric/flexric/build/examples/xApp/c/monitor
./xapp_kpm_moni
```

### 4. ns-3 scenario

```bash
cd /home/user/ns-O-RAN-flexric/mmwave-LENA-oran
./ns3 run "scratch/scenario-zero-sionna-kyunghee.cc \
  --sionnaServerIp=tcp://localhost:5556 \
  --N_MmWaveEnbNodes=2 \
  --N_Ues=3 \
  --CenterFrequency=3.5e9 \
  --Bandwidth=20e6 \
  --gnbPositions=contrib/sionna/examples/kyunghee-demo-gnbs.csv \
  --sumoTrace=contrib/sionna/examples/kyunghee-demo-ues.csv \
  --simTime=0.11 \
  --trafficStart=0.001 \
  --enableE2=true \
  --e2TermIp=127.0.0.1 \
  --realtime=true \
  --requireTraffic=false"
```

For a GUI/RT-only diagnostic without FlexRIC, use `--enableE2=false` and omit
`--realtime=true`. The RIC-TaaP Studio web KPI views (`localhost:8000` and
`localhost:3000`) remain a separate application from this Polyscope window.

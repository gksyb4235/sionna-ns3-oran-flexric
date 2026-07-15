# sionna-ns3-oran-flexric

Sionna RT(레이트레이싱) + ns-3(O-RAN, `mmwave`/EN-DC 모듈) + FlexRIC 기반 디지털트윈 AI-RAN 실험 환경.

## 배경

이 프로젝트는 [`sionna-oai-flexric-smo`](https://github.com/gksyb4235)에서 시작됐다. 그 프로젝트는 경희대 국제캠퍼스 실측 3D 씬(Sionna RT) + OAI 5G SA(rfsim) + FlexRIC으로 per-UE CIR 주입까지 구현했지만, **OAI-gNB를 CU/DU로 쪼개 멀티-gNB × 멀티-UE로 확장하는 것은 OAI rfsimulator 자체의 구조적 한계**(동시 2-cell 링크를 지원하는 rfsimulator 버전보다 이 프로젝트가 고정한 버전이 오래됨)로 막혔다. 이를 우회하기 위해 gNB/UE PHY 스택 전체를 [Orange-OpenSource/ns-O-RAN-flexric](https://github.com/Orange-OpenSource/ns-O-RAN-flexric)(ns-3 + e2sim + FlexRIC 기반 RIC-TaaP)로 대체한 것이 이 저장소다.

- `mmwave-LENA-oran`의 레거시 `mmwave` 모듈은 Scenario Zero/Handover xApp/ES xApp 등 모든 기존 RIC-TaaP 데모가 E2/KPM/RC까지 이미 배선돼 있고, **`--N_MmWaveEnbNodes`/`--N_Ues`로 멀티-gNB × 멀티-UE가 이미 지원**된다 — OAI에서 막혔던 지점이 여기선 애초에 문제가 안 된다.
- `mmwave-LENA-oran/src/sionna`에 [robpegurri/ns3-rt](https://github.com/robpegurri/ns3-rt) 기반 Sionna RT ↔ ns-3 브리지가 이미 vendored되어 있다 (UDP 8103, `PropagationLossModel::CalcRxPower()`를 투명하게 가로채는 방식).
- 목표: 기존 `sionna-oai-flexric-smo`의 Polyscope GUI + Sionna RT 물리 RSRP 계산은 그대로 유지하면서, gNB/UE PHY와 CIR 주입 경로만 ns-3 + FlexRIC으로 교체한 종단간 파이프라인 구축.

## 구조

```
sionna-ns3-oran-flexric/
├── mmwave-LENA-oran/     # ns-3 (레거시 mmwave 모듈 + 5G-LENA nr 모듈 + oran-interface(E2/KPM/RC) + vendored Sionna RT 브리지)
├── e2sim-kpmv3/          # O-RAN E2 termination 시뮬레이터 (E2AP v1.01 / KPM v3.00)
├── docs/, fig/           # 원본 Orange-OpenSource 문서/그림
└── README.upstream.md    # 원본 Orange-OpenSource README (참고용, 원문 그대로 보존)
```

**이 저장소에 포함되지 않은 것** (별도 설치 필요, `.gitignore` 참고):
- `flexric/` — EURECOM FlexRIC. 자체 GitLab remote를 갖는 독립 프로젝트라 형제 디렉토리로 별도 clone.
- `ns3-sionna/` — Sionna RT 브리지용 Python venv. `requirements.txt`로 재현.
- `mmwave-LENA-oran/build/`, `cmake-cache/`, `e2sim-kpmv3/e2sim/build/` — 빌드 산출물 (수 GB, 매번 로컬에서 재빌드).

## 설치

### 1. 필수 패키지 (Ubuntu)

```bash
sudo apt-get update
sudo apt-get install -y build-essential git cmake libsctp-dev autoconf automake libtool bison flex libboost-all-dev
sudo apt-get install -y g++-13 python3.12 python3.12-venv libc6-dev
```

### 2. e2sim 빌드 + 시스템 설치

```bash
cd e2sim-kpmv3/e2sim
mkdir build
sudo ./build_e2sim.sh 2
```

`contrib/oran-interface`(ns-3)가 이때 `/usr/local/include/e2sim`에 설치되는 ASN.1 헤더를 필요로 하므로, **ns-3 빌드보다 반드시 먼저** 실행해야 한다.

### 3. ns-3 빌드

```bash
cd mmwave-LENA-oran
./ns3 configure
./ns3 build
```

`--enable-examples`는 붙이지 말 것 — 이 fork는 `lte` 모듈과 `mmwave` 모듈이 서로를 참조하는 구조라, `src/lte/examples/`의 순정 ns-3 예제(`lena-cqi-threshold` 등)를 함께 빌드하면 공유 라이브러리 순환 참조로 링크가 실패한다. `scratch/`의 실제 시나리오들은 이 플래그와 무관하게 항상 빌드된다.

### 4. FlexRIC 빌드 + 설치

```bash
git clone https://gitlab.eurecom.fr/mosaic5g/flexric.git && cd flexric
git checkout oie-ric-taap-xapps
mkdir build && cd build
cmake .. -DE2AP_VERSION=E2AP_V1 -DKPM_VERSION=KPM_V3_00
make -j8
sudo make install
```

`examples/xApp/c/monitor/CMakeLists.txt`의 `add_subdirectory(RRC_MESSAGES)`와 그에 딸린 `xapp_rc_moni` 타겟이 `asn1c` 실행파일을 못 찾아 빌드가 깨질 수 있다 — 해당 블록을 주석 처리하고 다시 빌드하면 된다 (다른 xApp들엔 영향 없음).

### 5. Python 환경 (Sionna RT 브리지)

```bash
python3.12 -m venv ns3-sionna
source ns3-sionna/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

`sionna-oai-flexric-smo`에서 이미 검증된 것과 동일한 버전 조합(Sionna RT 2.0.1)을 그대로 맞춘다.

## 알려진 이슈 / 트러블슈팅

- **`.sh` 스크립트 실행권한 소실**: 저장소를 다른 머신으로 옮기는 과정에 따라 `.sh` 스크립트들의 실행 비트가 빠질 수 있다 (`find . -name "*.sh" ! -perm -u+x -exec chmod +x {} \;`로 일괄 복구).
- **e2sim ASN.1 소스 대소문자 충돌**: 대소문자 구분 없는 파일시스템을 거쳐 복사되면 `GlobalgNB-ID.h`/`GlobalGNB-ID.h`처럼 대소문자만 다른 파일 쌍이 서로 덮어써질 수 있다. `git status`로 deleted/modified 쌍이 보이면 `git restore`로 원복.
- **`lte`/`mmwave` 모듈이 `contrib/oran-interface` 심볼을 못 찾는 링크 에러**: 두 모듈의 `CMakeLists.txt` `LIBRARIES_TO_LINK`에 `${liboran-interface}`가 누락돼 있었음 (이 fork 자체의 버그, 이미 수정됨).
- **GUI(RIC-TaaP Studio) Docker의 InfluxDB에 데이터가 안 뜸**: `GUI/docker-compose.yml`의 influxdb 서비스가 커스텀 `command:`로 기본 entrypoint를 우회해서, `INFLUXDB_DB=influx` 자동 생성 로직이 스킵된다. `docker exec <influxdb-container> influx -execute "CREATE DATABASE influx"`로 수동 생성 필요. 또한 `gui_trigger.py`가 띄우는 `sim_data_pusher.py`는 시스템 Python에 `pip3 install influxdb`가 돼 있어야 한다.

## 원본 프로젝트

이 저장소는 [Orange-OpenSource/ns-O-RAN-flexric](https://github.com/Orange-OpenSource/ns-O-RAN-flexric)의 fork/vendor다. 원본 README는 [`README.upstream.md`](README.upstream.md)에 그대로 보존했다.

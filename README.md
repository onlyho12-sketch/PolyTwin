# PolyTwin — 자동차 차체 자동 폴리싱 디지털 트윈

**Doosan M0609 6축 로봇팔** 3대(천장 C · 측면 SL/SR)가 차체를 자동 폴리싱하는
**NVIDIA Isaac Sim** 기반 디지털 트윈입니다.

규칙 기반 시뮬레이션에서 출발해 **모방학습(BC) → 강화학습 잔차 정책 → 광택(Gloss) 검증 →
관제 웹 콘솔**까지 하나의 파이프라인으로 연결되어 있습니다.

```
스캔 ──▶ 경로 생성 ──▶ 폴리싱 시뮬 ──▶ 학습(BC/RL) ──▶ 광택 검증 ──▶ 웹 콘솔
scan.py  path_generator  polishing_v5   learning/      gloss_test/   polytwin_ui_hh/
                              ▲              │
                              └── 잔차 정책 ──┘
                                (rl_bridge.py)
```

---

## 1. 구성 요소

저장소는 독립적으로 실행 가능한 네 덩어리로 나뉩니다.

| 디렉터리 | 역할 | 문서 |
|---|---|---|
| [scripts/](scripts/) | Isaac Sim 폴리싱 시뮬레이션 (원코드) | 본 문서 |
| [learning/](learning/) | 모방학습(BC) · 강화학습(PPO) · 공정조건 최적화(BO) | [learning/README.md](learning/README.md) |
| [gloss_test/](gloss_test/) | 20° Gloss 디지털 트윈 검증 실험 | [gloss_test/README.md](gloss_test/README.md) |
| [polytwin_ui_hh/](polytwin_ui_hh/) | 관제·설정 웹 콘솔 (Node + Vercel) | [polytwin_ui_hh/README.md](polytwin_ui_hh/README.md) |

### 파이프라인 단계

| 단계 | 스크립트 | 실행기 | 입력 → 출력 |
|---|---|---|---|
| ① 스캔 | `scripts/scan.py` | `isaac_python` | USD 오브젝트 → `scan_result/{obj}/points/*.ply` |
| ② 경로 생성 | `scripts/path_generator.py` | `python3` | `*.ply` → `path.npy` |
| ③ 폴리싱 | `scripts/polishing_v5.py` | `isaac_python` | `path.npy` → 시뮬레이션 + 힘 로그 |
| ④ 학습 | `learning/bc/`, `learning/rl/` | venv `python` | 힘 로그 → 정책 체크포인트 |
| ⑤ 광택 검증 | `gloss_test/run_*.sh` | `isaac_python` | RL 출력 → RTX 렌더 기반 광택 지표 |
| ⑥ 관제 UI | `polytwin_ui_hh/` | `node` | KPI/궤적 → 웹 콘솔 |

### 시뮬레이션 버전

- **v1** (`polishing_v1.py`) — 단일 로봇 폴리싱.
- **v4** (`polishing_v4.py`) — 4대 모드.
- **v5** (`polishing_v5.py` + [polishing_v5_modules/](scripts/polishing_v5_modules/)) — **현재 기준.**
  레일 + 측면(SL/SR) + 천장(C) 다중 로봇.

  | 모듈 | 역할 |
  |---|---|
  | `common.py` | 상수 · 설정 · USD 로딩 유틸 |
  | `agent.py` | 로봇별 제어 (RMPFlow · 접촉력) |
  | `runner.py` | 씬 구성 · 메인 루프 |
  | `bootstrap.py` | CLI 진입점 |
  | `pad_contact.py` | 패드 접촉 모델 |
  | `rl_bridge.py` | **학습 스택 연결** — 셀 격자 · 잔차 정책 · 셀별 품질 판정 |
  | `ros_publisher.py` | ROS2 토픽 발행 |
  | `visualization.py` | 디버그 시각화 |

### 제어 개요

- **모션** — [rmpflow/](rmpflow/) 의 `RMPFlowController`로 End-Effector를 `path.npy`에 추종.
- **접촉력** — 실측 위치 기반 **가상 스프링** 모델로 법선 방향 누름 힘 제어(목표 ≈ 1.5N).
  강체 충돌 슬램을 피하려고 패드/로봇 물리 충돌은 끄고 가상힘으로 처리합니다.
- **패드 회전** — USD의 `RevoluteJoint`(`pad_joint`) 속도 구동.
- **잔차 정책(선택)** — `POLISH_RL=1` 이면 챔피언 정책이 20 Hz로
  `[Δforce ±30%, Δfeed ±50%]` 를 얹습니다. 원코드 훅은 `agent.py` 두 줄뿐입니다.

---

## 2. 실행 환경

| 항목 | 사양 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| 시뮬레이터 | NVIDIA **Isaac Sim 6.0.1** (standalone `python.sh`) |
| ROS | ROS2 Humble (`/opt/ros/humble`) — 토픽 발행용, 선택 |
| Python (시스템) | 3.10 |
| Python (학습) | `~/isaacsim_venv` — torch 2.7 + CUDA |
| GPU | NVIDIA RTX 계열 (RTX 5080 Laptop 기준 개발) |
| 웹 콘솔 | Node.js **22.5+** (`node:sqlite` 내장 모듈 사용) |

Isaac Sim 스크립트는 반드시 `isaac_python`으로 실행해야 하며 시스템 `python3`으로는 동작하지 않습니다.

```bash
# Isaac Sim 경로 지정 — 스크립트가 $ISAAC_PYTHON 을 읽습니다 (기본값 ~/isaacsim-6.0.1/python.sh)
export ISAAC_PYTHON=~/isaacsim-6.0.1/python.sh
alias isaac_python="$ISAAC_PYTHON"
```

---

## 3. 대상 장비 (가상)

| 장비 | 설명 |
|---|---|
| Doosan **M0609** 6축 협동로봇 ×3 | 폴리싱 로봇팔 (URDF/USD 포함) |
| OnRobot 샌더 + 폴리싱 패드 | End-Effector 공구 (`m0609_with_polisher.usd`) |
| Vention **518823** 텔레스코픽 리프트 | 측면/천장 로봇 받침대 (접힘 830 / 펼침 1700 mm) |
| 리니어 레일 시스템 | 다중 로봇 이송 (`Rail.usd`) |
| 가상 깊이 카메라 | 스캔용 (Isaac Sim Replicator) |
| 차량 모델 | `car.usd`, `car_small.usd`, BMW Z4 등 |

---

## 4. 설치

```bash
pip install -r requirements.txt          # numpy · scipy · matplotlib · Pillow · gmsh
```

pip로 설치하지 않는 것:

- **Isaac Sim 제공** — `isaacsim`, `omni.*`, `carb`, `pxr`
- **ROS2 Humble 제공** — `rclpy`, `std_msgs`, `sensor_msgs`
- **학습 스택** — torch 등은 `~/isaacsim_venv` 에 별도 구성

---

## 5. 사용 방법

### A. 폴리싱 시뮬레이션 보기 (권장 시작점)

```bash
bash scripts/run_v5_rl_view.sh              # Isaac Sim 창 — 3대가 차체를 닦는다
bash scripts/run_v5_rl_view.sh --headless   # 창 없이 검증용
```

잔차 정책(`POLISH_RL=1`)과 BO 레시피가 함께 적용되고, 종료 시 셀별 판정 CSV가
`learning/ui_bridge/out/` 에 남습니다.

> **Isaac 프로세스는 한 번에 하나만** 실행할 수 있습니다.
> 스크립트가 다른 프로세스를 감지하면 중단합니다.

### B. 단계별 실행

```bash
isaac_python scripts/scan.py --obj_name car            # ① 깊이 스캔
python3      scripts/path_generator.py --obj_name car  # ② 경로 생성
isaac_python scripts/polishing_v5.py --obj_name car    # ③ 폴리싱 (다중 로봇)
isaac_python scripts/polishing_v1.py --obj_name car    #    단일 로봇 버전
```

전체를 한 번에 돌리려면:

```bash
python3 scripts/main_pipeline.py car
```

### C. 학습

```bash
PY=~/isaacsim_venv/bin/python              # torch 2.7 + CUDA

$PY learning/bc/extract_dataset.py         # v5 힘 로그 → (state, action) 데이터셋
$PY learning/bc/train.py                   # 모방학습
$PY learning/rl/train_ppo_robot.py         # PPO 잔차 정책
```

로그를 누적하려면 실행 후 `scripts/force_log_rail_*.csv` 를
`learning/data/raw/<날짜>/` 로 복사하세요 — 자동으로 전부 읽고 중복은 제거합니다.
자세한 데이터 정의는 [learning/README.md](learning/README.md).

### D. 광택 검증

```bash
bash gloss_test/run_test.sh                        # 평면 Roughness sweep
bash gloss_test/run_vehicle_mesh_rtx_scan.sh       # BMW Z4 실제 Mesh RTX 검사
bash gloss_test/run_surface_generalization_benchmark.sh   # 6종 표면 일반화
```

> **주의** — 산출되는 `20° GU` 는 논문 앵커 기반 **proxy** 이며 실제 Gloss Meter로
> 보정한 값이 아닙니다. CSV의 `not_gu` · `proxy` ·
> `actual_gloss_meter_calibrated=false` 표기를 의도적으로 유지합니다.

### E. 관제 웹 콘솔

```bash
cd polytwin_ui_hh
npm run seed:data          # 최초 1회 — KPI JSON을 DB에 적재
node backend/server.js     # → http://127.0.0.1:8000
```

`python -m http.server` 로는 동작하지 않습니다 (로그인·계정 API 필요).
기본 관리자 계정은 `admin` / `polytwin2026` 이며 **배포 전 반드시 변경**하세요.
자세한 내용은 [polytwin_ui_hh/SERVER.md](polytwin_ui_hh/SERVER.md).

---

## 6. 디렉터리 구조

```
cacadaca/
├── README.md                  # (본 문서)
├── requirements.txt
├── scripts/                   # ① 시뮬레이션 원코드
│   ├── scan.py                #   깊이 스캔
│   ├── path_generator.py      #   경로 생성
│   ├── polishing_v5.py        #   다중 로봇 폴리싱 진입점
│   ├── polishing_v5_modules/  #   v5 모듈 (agent · runner · rl_bridge …)
│   └── run_v5_rl_view.sh      #   시뮬 + 잔차 정책 실행
├── learning/                  # ② 학습 스택
│   ├── bc/                    #   모방학습
│   ├── rl/                    #   PPO · 챔피언 정책 · 평가
│   ├── polytwin/              #   공정조건 최적화(BO)
│   └── ui_bridge/             #   시뮬 → UI 피드
├── gloss_test/                # ③ 광택 검증 실험
│   ├── scripts/               #   측정·집계 스크립트
│   └── run_*.sh               #   실험별 실행기
├── polytwin_ui_hh/            # ④ 관제 웹 콘솔
│   ├── frontend/              #   배포 루트 (화면 · 3D 자산 · 데이터셋)
│   ├── backend/               #   서버 · 라우트 · 인증
│   └── appendix/              #   문서 · 검수 도구
├── rmpflow/                   # RMPFlow 컨트롤러 + URDF
├── scan_obj/                  # 스캔 대상 오브젝트
├── scan_result/               # 스캔·경로 결과 (PLY, path.npy)
└── usd/env/                   # 씬 · 로봇 · 리프트 · 레일 USD 에셋
```

---

## 7. 저장소에 포함되지 않는 것

`.gitignore` 로 제외된 항목입니다. 로컬에서는 실행 시 생성되거나 그대로 남아 있습니다.

| 항목 | 이유 |
|---|---|
| `learning/rl/logs/` | 학습 로그 (8.6 GB) — 재생성 가능 |
| `frontend/데이터셋/sweep_traj_csv*` | sweep 원본 CSV (209 MB) — 화면은 여기서 뽑은 `데이터셋/*.json` 을 읽습니다 |
| `frontend/assets/{models,video}/_orig/` | 최적화본의 원본 마스터 — 배포는 `*.opt.glb` 사용 |
| `gloss_test/results/`, `usd/env/export_obj/` | 실험 산출물 |
| `__pycache__/`, `*.pyc`, `*.bak` | 캐시 · 백업 |
| `scripts/status_log.txt`, `force_log_rail_*.csv` | 실행마다 덮어쓰이는 런타임 로그 |

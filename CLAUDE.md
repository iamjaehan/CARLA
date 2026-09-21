# CARLA trajectory visualization

## 목표
Julia로 미리 계산한 trajectory(위치/방향/속도)를 CARLA 상에서 시각화. 실시간 반영 불필요 —
trajectory를 먼저 다 구한 뒤 사후에 재생하는 방식. discrete하게 끊긴 waypoint를 그대로 재생하고,
interpolation은 나중 과제로 미룸. CARLA 자체의 vehicle physics/dynamics는 꺼두고 위치를 강제로
박아넣는 v0에서 시작, 이후 바퀴 회전/조향 시각화/기울임 등 현실감 있는 표현을 추가할 예정.
하나 이상의 agent(멀티에이전트 시나리오 포함)를 같은 타임라인으로 재생할 수 있음.

## 구조
```
config.yaml                  CARLA 서버 접속 정보, 좌표계 변환, 재생 옵션
requirements.txt             pip 의존성 (carla는 서버 버전과 정확히 맞춰야 함)
data/example_trajectory.csv  t,agent_id,x,y,z,yaw,speed 스키마 샘플 (단일 agent, 곡선 경로)
data/hospital_trajectory.csv ScholtesReducedGOOP.jl hospital 시나리오 실제 output (3 agent, 0.3초짜리 궤적)
julia/export_trajectory.jl   이 스키마로 CSV 내보내는 헬퍼 (표준 라이브러리만 사용, 의존성 없음)
python/trajectory_io.py      CSV -> Waypoint 리스트 로더, agent_id로 그룹핑
python/replay.py             CARLA 접속, agent별 차량 스폰, physics 끄고 waypoint마다 set_transform
```

다른 레포에도 연동 파일이 있음:
```
../ScholtesReducedGOOP.jl/examples/hospital_to_carla.jl
    hospital.jl (3-agent 병원 복도 시나리오)를 실행해서 나온 결과를
    CARLA/data/ 스키마로 export. 상태가 위치(x,y)뿐이라 yaw/speed는 각 스텝의
    control u=(ux,uy)에서 유도 (yaw=atan(uy,ux), speed=‖u‖). 실행:
    `cd ../ScholtesReducedGOOP.jl && julia --project=. examples/hospital_to_carla.jl`
    (output_path 인자 생략하면 기본으로 ../CARLA/data/hospital_trajectory.csv에 씀)
```

## 핵심 설계 결정
- `vehicle.set_simulate_physics(False)`로 CARLA dynamics를 끄고, 매 waypoint를
  `set_transform()`으로 그대로 반영하는 방식 (v0). `replay.py` 참고.
- 멀티에이전트: agent_id별로 waypoint를 그룹핑해서 각자 vehicle을 하나씩 스폰. 전체
  타임라인(모든 agent의 t를 합쳐 정렬한 값)을 돌면서, 각 agent는 자기 마지막 waypoint 값을
  그대로 hold(step function, interpolation 없음). 카메라는 개별 추적이 아니라 전체 agent
  중심의 top-down 뷰(`update_spectator_topdown`)로 통일.
- 좌표계 변환(`config.yaml`의 `coordinate_transform.flip_y`, `negate_yaw`)은
  Julia가 오른손 좌표계(x=forward, y=left, yaw=CCW, rad)를 쓴다는 **가정** 하에 기본값을
  잡아둔 것. CARLA는 왼손 좌표계(x=forward, y=right, yaw=CW, deg)라 y/yaw 부호를 뒤집음.
  **아직 실제 CARLA에서 검증 안 됨** — 처음 돌려볼 때 차량이 예상 방향대로 도는지 꼭 확인할 것.
- 재생 속도는 `time.sleep` 기반. `config.yaml`의 `playback.speed_factor` 또는
  `replay.py --speed-factor`로 배속 조절 (커맨드라인 인자가 우선). 나중에 synchronous
  mode(`fixed_delta_seconds`)로 바꾸면 더 결정론적으로 재생 가능.
- 속도는 `world.debug.draw_string`으로 차량 위에 `"{agent_id}: {speed} m/s"` 텍스트 표시만 함
  (v0). 화살표/바퀴 회전 등은 미구현.
- hospital 시나리오는 `DT=0.1`, `PLANNING_HORIZON=4` → 전체 재생 시간이 0.3초로 매우 짧음.
  `--speed-factor 0.05` 정도로 슬로모션 걸어야 눈으로 보임 (예:
  `python replay.py --trajectory ../data/hospital_trajectory.csv --speed-factor 0.05`).
- hospital 시나리오는 초기 위치들이 서로 1~4m 거리라, 스폰 지점이 CARLA 맵의 건물/도로 중간에
  겹치거나 지형 고도와 안 맞아서 `spawn_actor`가 실패할 수 있음 — **아직 실제 CARLA에서 검증
  안 됨**. 실패하면 좌표에 global offset을 주거나 맵의 뻥 뚫린 구역으로 이동시켜서 재시도.

## 원격 환경
- CARLA 서버(시뮬레이터 본체)는 별도 원격 머신에서 실행 (GPU 필요, macOS는 공식 미지원).
  접속: `ssh ji5332@ase-a71908.ece.utexas.edu`
- CARLA 설치 위치: 그 머신의 `~/Documents/carla`, 실행은 그 안의 `.sh` 스크립트
  (보통 `CarlaUE4.sh`, 정확한 파일명은 `ls ~/Documents/carla`로 확인).
- CARLA 버전: **0.9.16** → `requirements.txt`에 `carla==0.9.16`으로 고정해둠.
- **첫 테스트는 원격 머신에 직접 가서, 서버·클라이언트를 둘 다 localhost로 돌리는 걸 권장**
  (네트워크/방화벽/VNC 관련 변수를 다 제거하고 파이프라인 로직 자체만 검증하기 위함). 이 경우
  `config.yaml`의 `server.host`를 `127.0.0.1`로 바꿔야 함 — 아직 안 바꿔둠 (원격 주소로
  되어있음), 로컬 테스트 시작할 때 바꿀 것.
- 나중에 Mac에서 원격으로 접속하는 구조로 돌아갈 때 참고할 것들:
  - 모니터가 원격 머신에 물리적으로 연결돼 있어서, SSH만으로는 화면이 안 보임. 직접 그
    자리에서 보거나 `x11vnc`로 그 물리 화면을 미러링 (아직 설정 안 함).
  - 학교 네트워크 방화벽 때문에 2000번 포트로 직접 접속이 안 되면 SSH 터널
    (`ssh -L 2000:localhost:2000 -L 2001:localhost:2001 ji5332@ase-a71908.ece.utexas.edu`)로
    우회하고 `config.yaml`의 host를 `127.0.0.1`로.

## 아직 안 한 것 / 다음 단계
- [x] `config.yaml` server.host를 실제 원격 주소로 채우기 (로컬 테스트 시엔 127.0.0.1로 바꿀 것)
- [x] `requirements.txt`를 원격 서버 CARLA 버전(0.9.16)에 맞춤
- [x] 멀티에이전트 지원 (`replay.py`, `trajectory_io.py`) + hospital 시나리오 export
      (`hospital_to_carla.jl`) — 로컬에서 Julia 실행 및 CSV 파싱까지는 검증함, **CARLA
      연결 자체는 아직 한 번도 테스트 안 해봄** (로컬 Mac엔 carla 패키지 없음)
- [ ] 원격 머신에서 `~/Documents/carla`의 `.sh` 스크립트로 CARLA 서버 실행
- [ ] 같은 머신에서 `config.yaml`의 host를 `127.0.0.1`로 바꾸고 `pip install -r requirements.txt`
      후 `python replay.py`로 첫 접속 테스트 (example_trajectory.csv, 단일 agent부터)
- [ ] 접속 성공하면 좌표계 변환 가정(`flip_y`, `negate_yaw`) 검증 — 차량이 예상 방향대로 도는지 확인
- [ ] hospital_trajectory.csv로 멀티에이전트 테스트 (`--speed-factor 0.05`), spawn 실패 시
      좌표 offset 조정
- [ ] (선택) 이후 Mac ↔ 원격 구조로 돌아갈 때 SSH 터널 or x11vnc 설정
- [ ] Julia 쪽 실제 trajectory 생성 코드를 `export_trajectory.jl` 스키마에 맞춰 연결
- [ ] 바퀴 회전, 조향 시각화, 차체 기울임 등 현실감 개선 (v1)

## Git
- remote: `git@github.com:iamjaehan/CARLA.git` (origin), branch `master`

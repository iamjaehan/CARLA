# CARLA trajectory visualization

## 목표
Julia로 미리 계산한 trajectory(위치/방향/속도)를 CARLA 상에서 시각화. 실시간 반영 불필요 —
trajectory를 먼저 다 구한 뒤 사후에 재생하는 방식. discrete하게 끊긴 waypoint를 그대로 재생하고,
interpolation은 나중 과제로 미룸. CARLA 자체의 vehicle physics/dynamics는 꺼두고 위치를 강제로
박아넣는 v0에서 시작, 이후 바퀴 회전/조향 시각화/기울임 등 현실감 있는 표현을 추가할 예정.

## 구조
```
config.yaml                  CARLA 서버 접속 정보, 좌표계 변환, 재생 옵션
requirements.txt             pip 의존성 (carla는 서버 버전과 정확히 맞춰야 함)
data/example_trajectory.csv  t,x,y,z,yaw,speed 스키마 샘플
julia/export_trajectory.jl   이 스키마로 CSV 내보내는 예시 함수
python/trajectory_io.py      CSV -> Waypoint 리스트 로더
python/replay.py             CARLA 접속, 차량 스폰, physics 끄고 waypoint마다 set_transform
```

## 핵심 설계 결정
- `vehicle.set_simulate_physics(False)`로 CARLA dynamics를 끄고, 매 waypoint를
  `set_transform()`으로 그대로 반영하는 방식 (v0). `replay.py` 참고.
- 좌표계 변환(`config.yaml`의 `coordinate_transform.flip_y`, `negate_yaw`)은
  Julia가 오른손 좌표계(x=forward, y=left, yaw=CCW, rad)를 쓴다는 **가정** 하에 기본값을
  잡아둔 것. CARLA는 왼손 좌표계(x=forward, y=right, yaw=CW, deg)라 y/yaw 부호를 뒤집음.
  **아직 실제 CARLA에서 검증 안 됨** — 처음 돌려볼 때 차량이 예상 방향대로 도는지 꼭 확인할 것.
- 재생 속도는 `time.sleep` 기반 (`playback.speed_factor`로 배속 조절). 나중에 synchronous
  mode(`fixed_delta_seconds`)로 바꾸면 더 결정론적으로 재생 가능.
- 속도는 `world.debug.draw_string`으로 차량 위에 텍스트 표시만 함 (v0). 화살표/바퀴 회전 등은 미구현.

## 원격 환경
- CARLA 서버(시뮬레이터 본체)는 별도 원격 Linux/Windows 머신에서 실행 (GPU 필요, macOS는 공식
  미지원). 접속은 평소 쓰던 SSH.
- 원격 머신에 모니터가 물리적으로 연결돼 있어서, CarlaUE4 창은 그 모니터에 렌더링됨.
  SSH만으로는 화면이 안 보이므로 (SSH는 텍스트 세션), 직접 그 자리에서 보거나 필요하면
  `x11vnc`로 그 물리 화면을 미러링해서 원격에서 볼 수 있음 (아직 설정 안 함).
- `config.yaml`의 `server.host`는 아직 placeholder(`127.0.0.1`). 원격 서버 IP/포트와 설치된
  CARLA 버전(`pip install carla==X`가 정확히 맞아야 handshake 성공)을 확인해서 채워야 함.

## 아직 안 한 것 / 다음 단계
- [ ] `config.yaml` server.host를 실제 원격 IP로 채우기
- [ ] 원격 서버의 CARLA 버전 확인 후 `pip install carla==<버전>`
- [ ] `python replay.py`를 실제 CARLA 서버에 대고 첫 실행 — 좌표계 변환 가정 검증
      (로컬 Mac엔 carla 패키지가 없어서 지금까지는 syntax/CSV 로더만 확인한 상태)
- [ ] (선택) x11vnc 설정해서 Mac에서도 원격 화면 보기
- [ ] Julia 쪽 실제 trajectory 생성 코드를 `export_trajectory.jl` 스키마에 맞춰 연결
- [ ] 바퀴 회전, 조향 시각화, 차체 기울임 등 현실감 개선 (v1)

## Git
- remote: `git@github.com:iamjaehan/CARLA.git` (origin), branch `master`

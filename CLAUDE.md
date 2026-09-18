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
- CARLA 서버(시뮬레이터 본체)는 별도 원격 머신에서 실행 (GPU 필요, macOS는 공식 미지원).
  접속: `ssh ji5332@ase-a71908.ece.utexas.edu`
- CARLA 설치 위치: 그 머신의 `~/Documents/carla`, 실행은 그 안의 `.sh` 스크립트
  (보통 `CarlaUE4.sh`, 정확한 파일명은 `ls ~/Documents/carla`로 확인).
- CARLA 버전: **0.9.16** → `requirements.txt`에 `carla==0.9.16`으로 고정해둠.
- 원격 머신에 모니터가 물리적으로 연결돼 있어서, CarlaUE4 창은 그 모니터에 렌더링됨.
  SSH만으로는 화면이 안 보이므로 (SSH는 텍스트 세션), 직접 그 자리에서 보거나 필요하면
  `x11vnc`로 그 물리 화면을 미러링해서 원격에서 볼 수 있음 (아직 설정 안 함).
- `config.yaml`의 `server.host`는 `ase-a71908.ece.utexas.edu`, `port`는 CARLA 기본값 2000으로
  채워둠. 학교 네트워크 방화벽 때문에 2000번 포트로 직접 접속이 안 되면 SSH 터널
  (`ssh -L 2000:localhost:2000 -L 2001:localhost:2001 ji5332@ase-a71908.ece.utexas.edu`)로
  우회하고 `config.yaml`의 host를 `127.0.0.1`로 바꾸면 됨 — 아직 직접 접속 테스트는 안 해봄.

## 아직 안 한 것 / 다음 단계
- [x] `config.yaml` server.host를 실제 원격 주소로 채우기
- [x] `requirements.txt`를 원격 서버 CARLA 버전(0.9.16)에 맞춤
- [ ] 원격 머신에서 `~/Documents/carla`의 `.sh` 스크립트로 CARLA 서버 실행
- [ ] Mac에서 `pip install -r requirements.txt` 후 `python replay.py`로 첫 접속 테스트
      — 포트 2000 직접 접속 안 되면 위 SSH 터널 방식으로 재시도
- [ ] 접속 성공하면 좌표계 변환 가정(`flip_y`, `negate_yaw`) 검증 — 차량이 예상 방향대로 도는지 확인
- [ ] (선택) x11vnc 설정해서 Mac에서도 원격 화면 보기
- [ ] Julia 쪽 실제 trajectory 생성 코드를 `export_trajectory.jl` 스키마에 맞춰 연결
- [ ] 바퀴 회전, 조향 시각화, 차체 기울임 등 현실감 개선 (v1)

## Git
- remote: `git@github.com:iamjaehan/CARLA.git` (origin), branch `master`

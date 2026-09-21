"""
    export_trajectory(path, t, agent_id, x, y, z, yaw, speed)

CARLA 시각화 스크립트(`python/replay.py`)가 읽는 스키마로 trajectory를 CSV로 저장.
표준 라이브러리만 사용 (DataFrames/CSV.jl 의존성 없음).

좌표계는 오른손 좌표계 기준: x=forward, y=left, yaw=CCW(rad), 단위는 m, m/s.
다른 convention을 쓴다면 CARLA/config.yaml의 coordinate_transform 값을 맞춰서 조정.
여러 agent가 하나의 타임라인을 공유하면 agent_id로 구분 (문자열/숫자 아무거나, CSV에는 문자열로 기록됨).
"""
function export_trajectory(path::AbstractString, t, agent_id, x, y, z, yaw, speed)
    n = length(t)
    @assert n == length(agent_id) == length(x) == length(y) == length(z) == length(yaw) == length(speed)
    open(path, "w") do io
        println(io, "t,agent_id,x,y,z,yaw,speed")
        for k in 1:n
            println(io, "$(t[k]),$(agent_id[k]),$(x[k]),$(y[k]),$(z[k]),$(yaw[k]),$(speed[k])")
        end
    end
    return path
end

# 사용 예시 (실제 trajectory 계산 결과로 교체)
if abspath(PROGRAM_FILE) == @__FILE__
    t = collect(0.0:0.5:4.5)
    n = length(t)
    agent_id = fill(0, n)
    x = collect(0.0:3.0:13.5)
    y = zeros(n)
    z = zeros(n)
    yaw = zeros(n)
    speed = fill(6.0, n)

    export_trajectory(joinpath(@__DIR__, "..", "data", "example_trajectory.csv"),
                       t, agent_id, x, y, z, yaw, speed)
end

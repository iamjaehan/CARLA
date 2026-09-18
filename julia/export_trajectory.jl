using DataFrames
using CSV

"""
    export_trajectory(path, t, x, y, z, yaw, speed)

CARLA 시각화 스크립트(`python/replay.py`)가 읽는 스키마로 trajectory를 CSV로 저장.
좌표계는 오른손 좌표계 기준: x=forward, y=left, yaw=CCW(rad), 단위는 m, m/s.
다른 convention을 쓴다면 CARLA/config.yaml의 coordinate_transform 값을 맞춰서 조정.
"""
function export_trajectory(path::AbstractString, t, x, y, z, yaw, speed)
    df = DataFrame(t=t, x=x, y=y, z=z, yaw=yaw, speed=speed)
    CSV.write(path, df)
    return df
end

# 사용 예시 (실제 trajectory 계산 결과로 교체)
if abspath(PROGRAM_FILE) == @__FILE__
    t = 0.0:0.5:4.5
    x = collect(0.0:3.0:13.5)
    y = zeros(length(t))
    z = zeros(length(t))
    yaw = zeros(length(t))
    speed = fill(6.0, length(t))

    export_trajectory(joinpath(@__DIR__, "..", "data", "example_trajectory.csv"),
                       t, x, y, z, yaw, speed)
end

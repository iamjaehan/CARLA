"""
    export_trajectory(path, t, agent_id, x, y, z, yaw, speed)

Write a trajectory as CSV in the schema `python/replay.py` reads.
Stdlib only, no DataFrames/CSV.jl dependency.

Right-handed frame: x=forward, y=left, yaw=CCW (rad), units m, m/s.
Adjust CARLA/config.yaml's coordinate_transform if your convention differs.
Multiple agents can share one timeline via agent_id (any type, written as a string).
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

# Example usage (replace with a real trajectory)
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

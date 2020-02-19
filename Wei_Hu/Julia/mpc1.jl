using JuMP
using Ipopt
using GLPK
using FFMPEG
using Plots

solver = IpoptSolver(print_level=0)

# run_mpc() takes the robot's current position and velocity
# and returns an optimized trajectory of position, velocity, 
# and acceleration. 
function run_mpc(initial_position, initial_velocity)
    
model = Model(with_optimizer(Ipopt.Optimizer))

t = 0.1
    num_time_steps = 10
    max_acceleration = 0.5

    @variables model begin
        position[1:2, 1:num_time_steps]
        velocity[1:2, 1:num_time_steps]
        -max_acceleration <= acceleration[1:2, 1:num_time_steps] <= max_acceleration
    end

    # Dynamics constraints
    @constraint(model, [i=2:num_time_steps, j=1:2],
                velocity[j, i] == velocity[j, i - 1] + acceleration[j, i - 1] * t)
    @constraint(model, [i=2:num_time_steps, j=1:2],
                position[j, i] == position[j, i - 1] + velocity[j, i - 1] * t)

    # Cost function: minimize final position and final velocity
    @objective(model, Min, 
        100 * sum(position[:, end].^2) + sum(velocity[:, end].^2))

    # Initial conditions:
    @constraint(model, position[:, 1] .== initial_position)
    @constraint(model, velocity[:, 1] .== initial_velocity)

    status = optimize!(model)
    return JuMP.value.(position), JuMP.value.(velocity), JuMP.value.(acceleration)
end

# The robot's starting position and velocity
q = [1.0, 0.0]
v = [0.0, -1.0]

    # Plot the current position
    plot([q[1]], [q[2]], marker=(:hex, 10), xlim=(-1.1, 1.1), ylim=(-1.1, 1.1))
    
    # Run the MPC control optimization
    q_plan, v_plan, u_plan = run_mpc(q, v)
    
    # Draw the planned future states from the MPC optimization
    plot!(q_plan[1, :], q_plan[2, :], linewidth=5)
    
    # Apply the planned acceleration and simulate one step in time
t = 0.1
    u = u_plan[:, 1]
    v += u * t
    q += v * t


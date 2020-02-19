using JuMP
using Ipopt
using FFMPEG
using Plots
using IterTools

solver = IpoptSolver(print_level=0)

# run_mpc() takes the cart's current state
# and returns optimized input and state arrays.

function run_mpc(x0)

	model = Model(with_optimizer(Ipopt.Optimizer))

	A = [1.2 1;0 1]
	B = [0;1]

	Q = [1 0;0 1]
	R = 1

	cost = 0;

	nX = 2;
	nU = 2;

	N = 3;

	@variables model begin
        x[1:2, 1:N+1]
        u[1:2, 1:N]
        [-15;-15] <= x[1:2, 1:N+1] <= [15;15]
		-1 <= u[1,1:N] <= 1
	end

  	# Initial conditions
    @constraint(model, cons0, x[:,1] .== x0)
	@constraint(model, cons1, x[:,N+1] .== [0,0])

	for i in 1:N
		# Dynamics constraints
   		@constraint(model, cons[i=2:N+1],
                	x[1:2, i+1] == A*x[1:2, i] + B*u[1:2, i])

    	# Cost function: minimize final position and final velocity
        cost = cost + Q*sum(x[:, i+1].^2) + sum(u[:, i].^2)
	end

	@objective(model, Min, cost)


    status = optimize!(model)
    return JuMP.value.(x), JuMP.value.(u)
end

# The robot's starting position and velocity
x0 = [2.0, -1.0];

M = 25;
N = 3;

xOpt = zeros(2, M + 1)
xOpt[:, 1] = x0
uOpt = zeros(1, M);
xOpen = zeros(2, N+1, M);
predErr = zeros(2, M-N+1);

for k=1:M
	global x0
    xOpti, uOpti = run_mpc(x0);
    model = nothing;
    xOpen[:,:,k] = xOpti;
    xOpt[:,k+1] = xOpti[:,2];
	uOpt[:,k] = uOpti[1];
    x0 = xOpti[:,2];
end

# Plot the current position
plot(xOpt[1,:], xOpt[2,:])

using JuMP
using Ipopt

# ============ System Parameters ============
# The robot's starting position and velocity
x_now = [2.0, -1.0];

M = 25;
N = 3;

A = [1.2 1;0 1]
B = [0 0;0 1]

Q = [1 0;0 1]
R = 1

cost = 0;

nX = 2;
nU = 2;

N = 3;

# ============= Solver Init =============

# Model definition
model = Model(with_optimizer(Ipopt.Optimizer, print_level=0))

# Variables
# @variable(model, x[1:2, 1:N+1])
# @variable(model, u[1:2, 1:N])
@variables model begin
	x[1:2, 1:N+1]
	u[1:2, 1:N]
#	[-1500 -1500]' <= x[1:2, 1:N+1] <= [ 1000  1000]'
#	[-3 -pi/4]' <= u[1:2, 1:N] <= [ 3  pi/4]'
end

# Constraints
# @variable(model, x0[1:2])
@NLparameter(model, x0_1 == 0)
@NLparameter(model, x0_2 == 0)

@NLconstraint(model, cons01, x[1,1] == x0_1)
@NLconstraint(model, cons02, x[2,1] == x0_2)
@constraint(model, cons1, x[:,N+1] .== [0,0])

for i in 1:N
	# state constraints
	#=@constraint(model, -1500 <= x[1, i+1] <= 1000)
	@constraint(model, -1500 <= x[2, i+1] <= 1000)
	@constraint(model, -3  <= u[1, i] <= 3)
	@constraint(model, -pi/4 <= u[2, i] <= pi/4)=#
	# Dynamics constraints
	@constraint(model, A * x[1:2, i] + B * u[1:2, i] .== x[1:2, i+1] )
end

# Cost function: minimize final position and final velocity
@NLobjective(model, Min, sum(Q[1,1] * x[1, i+1]^2 + Q[2,2] * x[2, i+1]^2 for i in 1:N) + sum(R * (u[1, i]^2 + u[2, i]^2) for i in 1:N))

# =================== Iterations ================

xOpt = zeros(2, M + 1)
xOpt[:, 1] = x_now;
uOpt = zeros(2, M);
xOpen = zeros(2, N+1, M);
predErr = zeros(2, M-N+1);

for k=1:M
	global x_now, xOpen, xOpt, uOpt

	# fix(x0, x_now)
	set_value(x0_1, x_now[1])
	set_value(x0_2, x_now[2])

	status = optimize!(model)

	xOpti = JuMP.value.(x)
	uOpti = JuMP.value.(u)

	print("\nk: ", k)
	print("\nxOpti: ", xOpti)
	xOpen[:,:,k] = xOpti;
	xOpt[:,k+1] = xOpti[:,2];
	uOpt[:,k] = uOpti[:,1];
	x_now = xOpti[:,2];
end

print(xOpt)

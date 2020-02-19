function bikeFE(z, u)
    # z(1) = x
    # z(2) = y
    # z(3) = v
    # z(4) = phi
    # u(1) = a
    # u(2) = beta

    TS = 0.2;
    lf = 1.738;
    lr = 1.738;
    #beta = atan2(lr*tan(u(2)),(lf+lr));
    #zp = zeros(4,1);
	@variable(model, zp[1:4,1]);

    zp[1] = z[1]+TS*z[3]*cos(z[4]+u[2]);
    zp[2] = z[2]+TS*z[3]*sin(z[4]+u[2]);
    zp[3] = z[3]+TS*u[1];
    zp[4] = z[4]+TS*z[3]*sin(u[2])/lr;

	return zp

#     xp = x + TS*v*cos(psi+beta);
#     yp = y + TS*v*sin(psi+beta);
#     vp = v + TS*a;
#     psip = psi + TS*v*sin(beta)/lr;
end





function solve(N, z0, zN, zmin, zmax, umin, umax,TS)

    model = Model(with_optimizer(Ipopt.Optimizer))

    @variables model begin
		# Define state matrix
        z[1:4, 1:N+1]
		# Define input decision variables
        u[1:2, N]
	end

	# Initial conditions
    @constraint(model, cons0, z[:,1] .== z0)

	# objective function
	objective = 0;
	for j in 1:N
		objective =objective+ 10*norm(z[1,j]-zN[1])^2 + 10*norm(z[2,j]-zN[2])^2 + 0.01*norm(z[3,j]-zN[3])^2+0.01*norm(z[4,j]-zN[4])^2;

	# Dynamics constraints
	@constraint(model, cons[i=2:N+1],
				z[1:4, i+1] == A*x[1:2, i] + B*u[1:2, i])

#define state and input constraints

for i in 1:N
    constraints = [constraints zmin<=z[:,i]<=zmax umin<=u[:,i]<=umax z[:,i+1] == bikeFE(z[:,i],u[:,i])];
    if i <= N-1
        constraints = [constraints -2<=u(2,i+1)-u(2,i)<=2];
    end
end

constraints = [constraints z(:,1)==z0, zmin<=z(:,N+1)<=zmax];

# Solve
@objective(model, Min, objective)
status = optimize!(model)
if sol.problem == 0
    feas = 1;
    zOpt = value(z);
    uOpt = value(u);
    JOpt = value(objective);
end
end

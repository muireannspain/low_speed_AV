#load Waypoints and State Constraint points
load('Waypoints_new.mat')
load('Constraints.mat')
openfig('Map.fig')

# Define timestep
TS = 0.2;

waypoints = [X; Y];

# initial state
heading = atan((Y[2]-Y[1]) / (X[2] - X[1]));
z0 = [X[1]; Y[1]; 0; heading];

# end target
final = waypoints[:,end];

# initialization
z      = z0;
z_list = z;

#set reference velocity
v_ref = 5;

# Define horizon
N = 5;
i = 0;

#while the model has not reached within a certain tolerance of the end
#point
while norm(z[1:2] - final) > 2
	current_dis = vecnorm(waypoints-z[1:2]);
	current_idx = findall(x->x == minimum(current_dis),current_dis);
    #find the goal index
	goal_idx    = current_idx + 2;
    dist=z0[3]*TS;
    x_pos=z0[1]+cos(z0[4])*dist;
    y_pos=z0[2]+sin(z0[4])*dist;
    for i =current_idx+1:length(waypoints)
        x=waypoints(1,i);
        y=waypoints(2,i);
        if x>x_pos && y>y_pos
            goal_idx=current_idx+i;
            break
        end
    end
    # Define input constraints
    umax = [ 3  pi/4]';
    umin = [-3 -pi/4]';

    #calculate distance between points
    d_x=v_ref*TS;

    #calculate next n-points to interpolate
    x_interp=[];
    y_interp=[];
    for j=1:N
        x_interp=[x_interp z0(1)+cos(heading)*j*d_x];
        y_interp=[y_interp z0(2)+sin(heading)*j*d_x];
    end
    # Define goal state constraints (X,Y,V,Heading)
    goal = [waypoints(:, goal_idx); v_ref];
    disp(['Goal Index:', num2str(goal_idx)])

    # Define constraints
    zMax = [ 1000  1000  8  2*pi]';
    zMin = [-1500 -1500 -2  -2*pi]';
    for k=1:N
        zN(:,k) = [x_interp(k); y_interp(k); v_ref; heading];
    end
    disp(['Currently Solving for iter:', num2str(i)])
    [feas, zOpt, uOpt, JOpt] = CFTOC(N, z0, zN, zMin, zMax, umin, umax, TS);

    u = uOpt(:, 1);
    z = zOpt(:, 2);
#     # Plot the trajectory with the car visualized simultaneously
#         # Params of car
#     lf = 1.738;
#     lr = 1.738;
#     width = 2;
#     # Assume the length of the wheel is 0.5
#     lw = 0.5;
#     #figure
#     # front
#     x_f = z0(1)+lf*cos(z0(4));
#     y_f = z0(2)+lf*sin(z0(4));
#     # front left
#     x_f_l = x_f-width/2*sin(z0(4));
#     y_f_l = y_f+width/2*cos(z0(4));
#     # front right
#     x_f_r = x_f+width/2*sin(z0(4));
#     y_f_r = y_f-width/2*cos(z0(4));
#     # rear
#     x_r = z0(1)-lr*cos(z0(4));
#     y_r = z0(2)-lr*sin(z0(4));
#     # rear left
#     x_r_l = x_r-width/2*sin(z0(4));
#     y_r_l = y_r+width/2*cos(z0(4));
#     # rear right
#     x_r_r = x_r+width/2*sin(z0(4));
#     y_r_r = y_r-width/2*cos(z0(4));
#     # draw the car
# #     plot (x_f_l,y_f_l,"*")
# #     plot (x_f_r,y_f_r,"*")
# #     plot (x_r_l,y_r_l,"*")
# #     plot (x_r_r,y_r_r,"*")
#     hold all
#     h1=line([x_f_l,x_f_r],[y_f_l,y_f_r],'Color','blue')
#     h2=line([x_f_l,x_r_l],[y_f_l,y_r_l],'Color','blue')
#     h3=line([x_r_l,x_r_r],[y_r_l,y_r_r],'Color','blue')
#     h4=line([x_f_r,x_r_r],[y_f_r,y_r_r],'Color','blue')
#     pause(1);
#     # calculate the wheel
#     # left wheel
#     w_f_l_x = x_f_l+lw/2*cos(z0(4)-u(2,1));
#     w_r_l_x = x_f_l-lw/2*cos(z0(4)-u(2,1));
#     w_f_l_y = y_f_l+lw/2*sin(z0(4)-u(2,1));
#     w_r_l_y = y_f_l-lw/2*sin(z0(4)-u(2,1));
#     # right wheel
#     w_f_r_x = x_f_r+lw/2*cos(z0(4)-u(2,1));
#     w_r_r_x = x_f_r-lw/2*cos(z0(4)-u(2,1));
#     w_f_r_y = y_f_r+lw/2*sin(z0(4)-u(2,1));
#     w_r_r_y = y_f_r-lw/2*sin(z0(4)-u(2,1));
# #     w_f_x = x_f+lw/2*cos(z0(4)-u(2,1));
# #     w_f_y = y_f+lw/2*sin(z0(4)-u(2,1));
# #     w_r_x = x_f-lw/2*cos(z0(4)-u(2,1));
# #     w_r_y = y_f-lw/2*sin(z0(4)-u(2,1));
#     # Draw the wheel
#     h5=line ([w_f_l_x,w_r_l_x],[w_f_l_y,w_r_l_y],'Color','red')
#     h6=line ([w_f_r_x,w_r_r_x],[w_f_r_y,w_r_r_y],'Color','red')
#     h=[h1,h2,h3,h4,h5,h6];
    z_list = [z_list z];
    u_list = [u_list u];
    z0 = z;
    i = i + 1;
    pause(1)
    hold on
    plot(z_list(1,:), z_list(2,:), 'm', 'linewidth', 2)
    #delete(h)
end
save OutputTrajectory z_list u_list

function CFTOC(N, z0, goal, zmin, zmax, umin, umax, TS)

	# Define state matrix
	z = sdpvar(4,N+1);
	# assign(z(:,1),z0);

	# Define input decision variables
	u = sdpvar(2,N);

	#define objective function
	objective=0;

	for j=1:N
		objective = objective + 5*(z(1, j) - goal(1,j))^2 + 5*(z(2, j) - goal(2,j))^2 + 1 * (z(3, j) - goal(3,j))^2;
		objective = objective + 0.1 * u(1, j)^2 + 0.1 * u(2,j)^2;
	end

	#define state and input constraints
	constraints = [];

	for i = 1:N
	    constraints = [constraints zmin<=z(:,i)<=zmax];
	    constraints = [constraints umin<=u(:,i)<=umax];
	    constraints = [constraints z(:,i+1) == bikeFE(z(:,i),u(:,i))];
	    if i <= N-1
	        constraints = [constraints -pi/10<=u(2,i+1)-u(2,i)<=pi/10];
	    end
	end

	constraints = [constraints z(:,1)==z0, zmin<=z(:,N+1)<=zmax];

	# Set options for YALMIP and solver
	options = sdpsettings('verbose', 0, 'solver', 'ipopt');
	# Solve
	sol = optimize(constraints, objective, options);
	if sol.problem == 0
	    feas = 1;
	    zOpt = value(z);
	    uOpt = value(u);
	    JOpt = value(objective);
	else
	    feas=0;
	    zOpt = [];
	    uOpt = [];
	    JOpt = value(objective);
	end
end

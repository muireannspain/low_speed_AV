#!/usr/bin/env julia

# This version extract the MPC model out
using JuMP
using Ipopt
using CSV
# using IterTools
# using LinearAlgebra
using RobotOS
# @rosimport std_msgs.msg: Float64MultiArray
@rosimport cloud_msgs.msg: state
rostypegen()
# using std_msgs.msg
using cloud_msgs.msg


# Import waypoints
global arr = CSV.read("/home/uav/catkin_ws/src/path_follower/src/RealWaypoints.csv")
global waypoints = (convert(Matrix{Float64}, arr))'
X = waypoints[1,:]
Y = waypoints[2,:]
# X = [-126.0500 -122.6668 -119.2837 -115.9005 -112.5174 -109.1342 -105.7511 -102.3679  -98.9847  -95.6016  -92.2184  -88.8353  -85.4521  -82.0689  -78.6858  -75.3026  -71.9195  -68.5363  -65.1532  -61.7700]
# Y = [-18.4200  -17.8989  -17.3778  -16.8567  -16.3356  -15.8145  -15.2934  -14.7723  -14.2512  -13.7301  -13.2089  -12.6878  -12.1667  -11.6456  -11.1245  -10.6034  -10.0823   -9.5612   -9.0401   -8.5190]

# Define timestep
TS = 0.2;

# global waypoints = [X; Y];

# initial state
heading = atan((Y[2]-Y[1]) / (X[2] - X[1]));
global z0 = [X[1]; Y[1]; 0; heading];

# end target
global final = waypoints[:,end];

# initialization
# global z  = z0;
global z_list = z0;

#set reference velocity
global v_ref = 5;

# Define horizon
N = 10;
global i = 0;

# Define input constraints
umax = [ 3  pi/4]';
umin = [-3 -pi/4]';

# Define constraints
zmax = [ 1000  1000  8  2*pi]';
zmin = [-1500 -1500 -2  -2*pi]';

# -------------------------------------------------------------------
#                         SET UP CFTOC MODEL
# -------------------------------------------------------------------

println("Creating CFTOC Model...")

model = Model(solver = IpoptSolver(print_level=0))

# Define variables
@variables model begin
z[1:4, 1:N+1]
u[1:2, 1:N]
end

# Initialize NLparameter
@NLparameter(model, z0_1 == z0[1])
@NLparameter(model, z0_2 == z0[2])
@NLparameter(model, z0_3 == z0[3])
@NLparameter(model, z0_4 == z0[4])
@NLparameter(model, goal_1 == 0)
@NLparameter(model, goal_2 == 0)
@NLparameter(model, goal_3 == 0)
# Constraints of initial state
@NLconstraint(model, cons01, z[1,1] == z0_1)
@NLconstraint(model, cons02, z[2,1] == z0_2)
@NLconstraint(model, cons03, z[3,1] == z0_3)
@NLconstraint(model, cons04, z[4,1] == z0_4)


for j in 1:N
    # Dynamics constraints
    @NLconstraint(model, z[1,j]+TS*z[3,j]*cos(z[4,j]+u[2,j]) == z[1, j+1])
    @NLconstraint(model, z[2,j]+TS*z[3,j]*sin(z[4,j]+u[2,j]) == z[2, j+1])
    @constraint(model, z[3,j]+TS*u[1,j] == z[3, j+1])
    @NLconstraint(model, z[4,j]+TS*z[3,j]*sin(u[2,j])/1.738 == z[4, j+1])
    # Input constraints
    @constraint(model, umin[1] <= u[1, j] <= umax[1])
    @constraint(model, umin[2] <= u[2, j] <= umax[2])
    # state constraints
    @constraint(model, zmin[1] <= z[1, j+1] <= zmax[1])
    @constraint(model, zmin[2] <= z[2, j+1] <= zmax[2])
    @constraint(model, zmin[3] <= z[3, j+1] <= zmax[3])
    @constraint(model, zmin[4] <= z[4, j+1] <= zmax[4])
end

# Cost function
@NLobjective(model, Min, sum(5*(z[1, j] - goal_1)^2 + 5*(z[2, j] - goal_2)^2 + 1 * (z[3, j] - goal_3)^2 + 0.1 * u[1, j]^2 + 0.1 * u[2,j]^2 for j in 1:N))

println("Done")

# global tmp
tmp = 0

function loop(pub_obj)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        # ----------------------------------------MPC-------------------------
        println("Start Looping")
        println("tmp", tmp)

    	global current_dis = zeros(1,length(X))
        # println("Length of X:",length(X))
    	#while the model has not reached within a certain tolerance of the end
    	#point
    	if norm(z0[1:2] - final) > 0.2

            # println("z[1,2]",z[1:2],"final",final)
            # println("norm:",norm(z[1:2] - final))
    		global final, waypoints, v_ref, i, z0
    		for l in 1:length(X)
    			global z0, waypoints, current_dis
    			current_dis[l] = norm(waypoints[:,l]-z0[1:2])
    		end
    		current_idx = (find(x->x == minimum(current_dis),current_dis))[1];
            # println("minimum of current_dis:",minimum(current_dis))
    		# println("Current index:", current_idx)
    		goal_idx    = current_idx + 2;
            # Define goal state constraints (X,Y,V,Heading)
            goal = [waypoints[:, goal_idx]; v_ref];
            # println("Goal Index:",goal_idx)
    		if goal_idx > length(X)
    			break
    		end

    		# println("Currently Solving for iter:",i)

            # -----------------------------------------------------------
            #                      RUNNING MPC
            # -----------------------------------------------------------
    		# Solve
    		# status = optimize!(model)
    		status = solve(model)

    		# zOpti = JuMP.value.(z)
    		zOpti = getvalue(z)
    		uOpti = getvalue(u)
    		JOpti = getobjectivevalue(model)


    	    u_command = uOpti[:, 1];
    	    z0 = zOpti[:, 2];

    	    global z_list = [z_list, z0];
    	    i = i + 1;

            z_u = [z0;u_command]
            println(z_u)
            #
    	    # pub_data = Float64MultiArray()
    	    # pub_data.data = z_u
    	    # publish(pub_obj, pub_data)
    		# println("Published")

    	end
    	#--------------------------------------MPC end--------------------------
        rossleep(loop_rate)
    end
end

function callback(msg)
    global tmp
    tmp = msg.x
    # setvalue()
end
#
function main()
    init_node("rosjl_example")
    # tmp = state()
    # pub = Publisher{Float64MultiArray}("pts", queue_size=10)
    # pub = Publisher{state}("test",queue_size=10)
    pub = 0
    sub = Subscriber{state}("localization", callback, queue_size=10)
    # spin()
    loop(pub)
end
#
if ! isinteractive()
    main()
end

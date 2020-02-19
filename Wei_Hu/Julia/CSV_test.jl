using CSV
using LinearAlgebra

A = CSV.read("C:\\Users\\80954\\OneDrive\\Documents\\GitHub\\low_speed_AV\\Muireann\\Waypoints.csv")
arr = (convert(Matrix{Float64}, A))'
final = arr[end,:]
X = arr[1,:]
Y = arr[2,:]
z = [1,2];
w = norm(arr[:,1]-z[1:2])

for l in 1:length(X)
    			global z, waypoints, current_dis
    			current_dis[l] = norm(waypoints[:,l]-z[1:2])
    		end

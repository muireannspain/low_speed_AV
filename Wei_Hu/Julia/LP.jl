using JuMP
using Ipopt

m = Model(with_optimizer(Ipopt.Optimizer))
@variable(m, x >=0)
@variable(m, y >=0)
@objective(m, Min, 10x + 26y)
@constraint(m, const1,  11x + 3y >=  21)
@constraint(m, const2,   6x + 20y >= 39)
status = optimize!(m)
println("Status = $status")
println("Optimal Objective Function value: ", objective_value(m))
println("Optimal Solutions:")
println("x = ", value(x))
println("y = ", value(y))
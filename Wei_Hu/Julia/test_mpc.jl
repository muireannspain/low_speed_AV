using JuMP
using Ipopt

model = Model(with_optimizer(Ipopt.Optimizer))
const_term = 1.0;
@variable(model, const_term)
@constraint(model, con, 2x <= const_term)

for i=1:2
    
end
@constraint(model, con[i=1:2,j=1:2], x[i,j] >= 2)

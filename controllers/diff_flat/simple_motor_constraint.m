function [A,B] = simple_motor_constraint(t0,tf,hz,a_min,a_max)

m = (tf-t0)*hz+1;
A_base = zeros(m,15);

for k = 1:m
    t = (1/hz)*(k-1)+t0;
    for j = 3:15
        A_base(k,j) = t^(j-3)/factorial(j-3);
    end    
end

A = [-A_base ; A_base];
B = [-a_min.*ones(m,1) ; a_max.*ones(m,1)];

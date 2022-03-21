clear 

x0 = sym('x0','real');
th = sym('th','real');
s = [x0 ; th];

c  = 1;
dt = 0.05;
N = 4;

% Brute Force
xk = x0;
X = zeros(1,N);
th_cand = 1.5;
for k = 2:N
    xk = dyn(xk,th,c,dt);
    X(k) = dyn(X(k-1),th_cand,c,dt);
end

dF_br  = jacobian(xk,s);
out_br = double(subs(dF_br,s,[1 ; 5]));

% Recursive
for k = 1:N
     
end

% Function
function x_upd = dyn(x,th,c,dt)
    x_upd = c + dt*(x^2+th);
end

function A = diff(x,dt)
    A = [2*dt*x dt];
end

function r = recursion(x,th,c,dt,k,N)
    if k > 0
        r = A
end
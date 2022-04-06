clear 

x0 = sym('x0','real');
th = sym('th','real');
s = [x0 ; th];

c  = 1;
dt = 0.1;
N = 10;
s0 = [1 ; 5];

% Brute Force
tic
xk = x0;
for k = 1:N
    xk = dyn(xk,th,c,dt);
end

dF_br  = jacobian(xk,s);
out_br = double(subs(dF_br,s,s0));
toc

% Recursive
tic
X = zeros(1,N);
X(1) = s0(1);
for k = 2:N
    X(k) = dyn(X(k-1),s0(2),c,dt);
end

out_rc = [recA(X,s0(2),c,dt,N), recB(X,s0(2),c,dt,N)];
toc

% Function
function x_upd = dyn(x,th,c,dt)
    x_upd = c + dt*(x^2+th);
end

function rA = recA(X,th,c,dt,k)
    df = 2*dt*X(k);

    if k > 1
        rA = df.*recA(X,th,c,dt,(k-1));
    else
        rA = df;
    end
end

function rB = recB(X,th,c,dt,k)
    dfA = 2*dt*X(k);
    dfB = dt;

    if k > 1
        rB = dfA.*recB(X,th,c,dt,(k-1))+dfB;
    else
        rB = dfB;
    end
end
clear 

% Setup
x = sym('x0',[2 1],'real');
th = sym('th','real');
s = [x ; th];

c  = 1;
dt = 0.1;

f = [sin(x(1))+x(2) ; c + dt*(x(2)^2+th)];
df = jacobian(f,s);
matlabFunction(f,'File','f_calc','vars',{s});
matlabFunction(df,'File','df_calc','vars',{s});

% Test
N = 20;
s0 = [0.1 ; 2 ; 0.3];

% Brute Force
tic
x_upd = x;
for k = 1:N
    x_upd = subs(f,x,x_upd);
end

df_br = jacobian(x_upd,s);
out_br = double(subs(df_br,s,s0))
toc

% Recursive
tic
X = zeros(2,N);
X(1:2,1) = s0(1:2);
for k = 2:N
    sk = [X(:,k-1) ; s0(3)];
    X(:,k) = subs(f,s,sk);
end
out_rc = recursion(X,s0(3),N)
toc

function r = recursion(X,th,k)
    sk = [X(:,k) ; th];
    if k > 1
        dfk = df_calc(sk);
        rk = [ recursion(X,th,k-1) ; 0.0 0.0 1.0];

        r = dfk*rk;
    else
        r = df_calc(sk);
    end
end
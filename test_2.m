clear 

% Setup
x = sym('x0','real');
th = sym('th','real');
s = [x ; th];

c  = 1;
dt = 0.1;

f = c + dt*(x^2+th);
df = jacobian(f,s);
matlabFunction(f,'File','f_calc','vars',{s});
matlabFunction(df,'File','df_calc','vars',{s});

% Test
N = 10;
s0 = [2 ; 0.3];

% Brute Force
x_upd = x;
for k = 1:N
    x_upd = subs(f,x,x_upd);
end

df_br = jacobian(x_upd,s);
out_br = double(subs(df_br,s,s0));

% Recursive
X = zeros(1,N);
X(1) = s0(1);
for k = 2:N
    sk = [X(k-1) ; s0(2)];
    X(k) = subs(f,s,sk);
end
out_rc = recursion(X,s0(2),N)

function r = recursion(X,th,k)
    sk = [X(k) ; th];
    if k > 1
        dfk = df_calc(sk);
        rk = recursion(X,th,k-1);

        r = [dfk(1)*rk(1) dfk(1)*rk(2)+dfk(2)];
    else
        r = df_calc(sk);
    end
end
% X = zeros(1,N);
% X(1) = s0(1);
% for k = 2:N
%     X(k) = dyn(X(k-1),s0(2),c,dt);
% end
% 
% out_rc = [recA(X,s0(2),c,dt,N), recB(X,s0(2),c,dt,N)]
% toc
% 
% % Function
% function x_upd = dyn(x,th,c,dt)
%     x_upd = c + dt*(x^2+th);
% end
% 
% function rA = recA(X,th,c,dt,k)
%     df = 2*dt*X(k);
% 
%     if k > 1
%         rA = df.*recA(X,th,c,dt,(k-1));
%     else
%         rA = df;
%     end
% end
% 
% function rB = recB(X,th,c,dt,k)
%     dfA = 2*dt*X(k);
%     dfB = dt;
% 
%     if k > 1
%         rB = dfA.*recB(X,th,c,dt,(k-1))+dfB;
%     else
%         rB = dfB;
%     end
% end

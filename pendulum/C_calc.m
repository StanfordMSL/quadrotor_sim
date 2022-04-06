function [C,X] = C_calc(x0,th,N)

n = size(x0,1);
m = size(th,1);

C = zeros(n,(n+m),N);
C(:,:,1) = [ A_calc(x0,th) B_calc(x0,th) ];

X = zeros(n,N);
xk = x0;
X(:,1) = xk;
for k = 1:N-1
    xk = x_calc(xk,th);
    Ak = A_calc(xk,th);
    Bk = B_calc(xk,th);
    
    C(:,:,k+1) = Ak*C(:,:,k) + [zeros(n,n) Bk];
    X(:,k+1) = xk;
end
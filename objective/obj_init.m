function J = obj_init(X,U,Q,R,xs)

N = size(U,2);
J = 0;

for k = 1:N
    xk = X(:,k)-xs;
    uk = U(:,k);
    Qk = Q(:,:,k);
    Rk = R(:,:,k);
    
    Jk = xk'*Qk*xk + uk'*Rk*uk;
    J = J + Jk;
end

xk = X(:,N);
Qk = Q(:,:,N);

Jk = xk'*Qk*xk;
J = J + Jk;

end
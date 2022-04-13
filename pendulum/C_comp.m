function C = C_comp(Xhat,th,m)

[n,N] = size(Xhat);

C = zeros(n,m,N);
C(:,:,1) = [ A_calc(Xhat(1:2,1),th) B_calc(Xhat(1:2,1),th) ];

for k = 1:N-1
    Ak = A_calc(Xhat(:,k),th);
    Bk = B_calc(Xhat(:,k),th);
    
    C(:,:,k+1) = Ak*C(:,:,k) + [zeros(n,n) Bk];
end
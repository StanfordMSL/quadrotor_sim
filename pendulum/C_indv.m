function C = C_indv(Xhat,th,m)

[n,N] = size(Xhat);

C = zeros(n,m,N);
C(:,:,1) = B_calc(Xhat(1:2,1),th);

for k = 1:N-1  
    C(:,:,k+1) = B_calc(Xhat(:,k),th);
end
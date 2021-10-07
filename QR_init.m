function [Q,R] = QR_init(nx,nu,N)

% qk = [100 ; 100 ; 0 ; 0 ; 0 ; 0];
qk = zeros(nx,1);
rk = ones(nu,1);

Qk = diag(qk);
Rk = diag(rk);

Q = repmat(Qk,[1 1 N+1]);
R = repmat(Rk,[1 1 N]);

end
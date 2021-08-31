function [La_u_o,La_u_c] = lagr_u(U,Ubar,conu,lamu,mudu,lqr)

% Unpack Some Stuff
Us  = lqr.Us;
N   = lqr.N;
Rn  = lqr.Rn;

% Initialize Variables
La_u_o = zeros(1,N);
La_u_c = zeros(1,N);

% Compute stagewise cost
for k = 1:N-1
    u = U(:,k);
    ub = Ubar(:,k);
    us = Us(:,k);
    cu = conu(:,k);
    ldu = lamu(:,k);
    mdu = mudu(:,k);
    
    La_u_o(1,k) = dCn_u(u,ub,us,Rn,1);

    La_u_c(1,k) = conu_cost(cu,ldu,mdu);
end

% Pad the last terms
La_u_o(1,N) = dCN_u(1);
La_u_c(1,N) = 0;

end
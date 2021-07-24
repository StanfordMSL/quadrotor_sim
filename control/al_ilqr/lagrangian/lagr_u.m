function [La_u_o,La_u_c] = lagr_u(U,Ubar,us,conu,lamu,mudu,T)

% Initialize Variables
N      = size(U,2);
La_u_o = zeros(1,N+1);
La_u_c = zeros(1,N+1);

% Compute stagewise cost
for k = 1:N
    u = U(:,k);
    ub = Ubar(:,k);
    cu = conu(:,k);
    ldu = lamu(:,k);
    mdu = mudu(:,k);
    
    if k < T
        La_u_o(1,k) = dCn_u(u,ub,us,1);
    else
        La_u_o(1,k) = dCT_u(u,ub,us,1);
    end

    La_u_c(1,k) = conu_cost(cu,ldu,mdu);
end

% Pad the last terms
La_u_o(1,N+1) = dCN_u(1);
La_u_c(1,N+1) = 0;

end
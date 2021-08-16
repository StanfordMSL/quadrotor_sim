function La_xup = lagr_xup(X,U,Xbar,Ubar,lqr)

% Unpack Some Stuff
xs  = lqr.xs;
us  = lqr.us;
N   = lqr.N;
Qn  = lqr.Qn;
QN  = lqr.QN;
Rn  = lqr.Rn;

% Initialize Variables
La_xup = zeros(2,N);

% Compute stagewise cost
for k = 1:N-1
    x  = X(:,k);
    u  = U(:,k);
    
    xb = Xbar(:,k);
    ub = Ubar(:,k);
    
    La_xup(1,k) = dCn_xu(x,u,xb,ub,xs,us,1);
    La_xup(2,k) = dCn_p(xb,ub,xs,us,Qn,Rn,1);
end

xb = Xbar(:,N);

La_xup(1,N) = dCN_xu(1);
La_xup(2,N) = dCN_p(xb,xs,QN,1);

end
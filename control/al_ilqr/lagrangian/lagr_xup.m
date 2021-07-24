function La_xup = lagr_xup(X,U,Xbar,Ubar,xs,us,T)

% Initialize Variables
N     = size(X,2);
La_xup = zeros(2,N);

% Compute stagewise cost
for k = 1:N-1
    x  = X(:,k);
    u  = U(:,k);
    
    xb = Xbar(:,k);
    ub = Ubar(:,k);
    
    if k < T
        La_xup(1,k) = dCn_xu(x,u,xb,ub,xs,us,1);
        La_xup(2,k) = dCn_p(xb,ub,xs,us,1);
    else
        La_xup(1,k) = dCT_xu(x,u,xb,ub,xs,us,1);
        La_xup(2,k) = dCT_p(xb,ub,xs,us,1);
    end
end

xb = Xbar(:,N);

La_xup(1,N) = dCN_xu(1);
La_xup(2,N) = dCN_p(xb,xs,1);

end
function [La_x_o,La_x_c] = lagr_x(X,Xbar,conx,lamx,mudx,lqr)

% Unpack Some Stuff
xs  = lqr.xs;
N   = lqr.T;
Qn  = lqr.Qn;
QN  = lqr.QN;

% Initialize Variables
La_x_o = zeros(1,N);
La_x_c = zeros(1,N);

% Compute stagewise cost
for k = 1:N
    x = X(:,k);
    xb = Xbar(:,k);
    cx = conx(:,k);
    ldx = lamx(:,k);
    mdx = mudx(:,k);
    
    if k < N
        La_x_o(1,k) = dCn_x(x,xb,xs,Qn,1);
    else
        La_x_o(1,k) = dCN_x(x,xb,xs,QN,1);
    end
    
    La_x_c(1,k) = conx_cost(cx,ldx,mdx);
end

end
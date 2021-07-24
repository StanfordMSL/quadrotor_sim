function [La_x_o,La_x_c] = lagr_x(X,Xbar,xs,conx,lamx,mudx,T)

% Initialize Variables
N     = size(X,2);
La_x_o = zeros(1,N);
La_x_c = zeros(1,N);

% Compute stagewise cost
for k = 1:N
    x = X(:,k);
    xb = Xbar(:,k);
    cx = conx(:,k);
    ldx = lamx(:,k);
    mdx = mudx(:,k);
    
    if k < T
        La_x_o(1,k) = dCn_x(x,xb,xs,1);
    elseif ((k >= T) && (k < N))
        La_x_o(1,k) = dCT_x(x,xb,xs,1);
    else
        La_x_o(1,k) = dCN_x(x,xb,xs,1);
    end
    
    La_x_c(1,k) = conx_cost(cx,ldx,mdx);
end

end
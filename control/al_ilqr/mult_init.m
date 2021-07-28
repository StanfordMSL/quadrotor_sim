function mult = mult_init(con)

% Unpack Some Stuff
N = size(con.cx,2);
nx = size(con.cx,1);
nu = size(con.cu,1);

% Initialize Multiplier Terms
mult.lamx = 0.*ones(nx,N);
mult.lamu = 0.*ones(nu,N);

mult.mux  = ones(nx,N);
mult.muu  = ones(nu,N);
mult.mudx = zeros(nx,N);
mult.mudu = zeros(nu,N);

% % Compute First Pass of Lagrangian
% [La_x_o,La_x_c] = lagr_x(X,X,xs,con.cx,lamx,mudx,T);
% [La_u_o,La_u_c] = lagr_u(U,U,us,con.cu,lamu,mudu,T);
% La_xup = lagr_xup(X,U,X,U,xs,us,T);
% 
% % Normalize It
% num = sum(La_x_o) + sum(La_u_o) + sum(sum(La_xup));
% den = sum(La_x_c) + sum(La_u_c);
% 
% if (den == 0 || num == 0)
%     coeff = 1;
% else
%     coeff =  num/den;
% end
% 
% % Re-initialize Multiplier Terms
% mux  = coeff.*mux;
% muu  = coeff.*muu;
% mudx = check_con(conx,lamx,mux,0);
% mudu = check_con(conu,lamu,muu,0);
% 
% % Compute Normalized Lagrangian
% [La_c.x_o,La_c.x_c] = lagr_x(X,X,xs,conx,lamx,mudx,T);
% [La_c.u_o,La_c.u_c] = lagr_u(U,U,us,conu,lamu,mudu,T);
% La_c.xup = lagr_xup(X,U,X,U,xs,us,T);
% 
% % Package it Out
% mult.lamx = lamx;
% mult.lamu = lamu;
% 
% mult.mux =  mux;
% mult.muu = muu;
% mult.mudx = mudx;
% mult.mudu = mudu;
% 
% La_c.obj = sum(La_c.x_o) + sum(La_c.u_o) + sum(sum(La_c.xup));
% La_c.con = sum(La_c.x_c) + sum(La_c.u_c);
% La_c.tot = La_c.obj + La_c.con;
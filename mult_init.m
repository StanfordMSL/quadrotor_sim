function [mult,La_c] = mult_init(X,U,xs,us,con,T)

% Unapack Some Stuff
N = size(con.c,2);
nx = con.nx;
nu = con.nu;

conx = con.c(1:nx,:);
conu = con.c(nx+1:end,:);

lamx = 0.*ones(nx,N);
mux  = ones(nx,N);
mudx = check_con(conx,lamx,mux,0);

lamu = 0.*ones(nu,N);
muu  = ones(nu,N);
mudu = check_con(conu,lamu,muu,0);

% Compute First Pass of Lagrangian
[La_x_o,La_x_c] = lagr_x(X,X,xs,conx,lamx,mudx,T);
[La_u_o,La_u_c] = lagr_u(U,U,us,conu,lamu,mudu,T);
La_xup = lagr_xup(X,U,X,U,xs,us,T);

% Normalize It
num = sum(La_x_o) + sum(La_u_o) + sum(sum(La_xup));
den = sum(La_x_c) + sum(La_u_c);

if (den == 0 || num == 0)
    coeff = 1;
else
    coeff =  num/den;
end

mux = coeff.*mux;
muu = coeff.*muu;
mudx = check_con(conx,lamx,mux,0);
mudu = check_con(conu,lamu,muu,0);

[La_x_o,La_x_c] = lagr_x(X,X,xs,conx,lamx,mudx,T);
[La_u_o,La_u_c] = lagr_u(U,U,us,conu,lamu,mudu,T);
La_xup = lagr_xup(X,U,X,U,xs,us,T);

% Package it Out
mult.lamx = lamx;
mult.lamu = lamu;
mult.mux =  mux;
mult.muu = muu;
mult.mudx = check_con(conx,lamx,mux,0);
mult.mudu = check_con(conu,lamu,muu,0);

La_c.x_o = La_x_o;
La_c.x_c = La_x_c;
La_c.u_o = La_u_o;
La_c.u_c = La_u_c;
La_c.xup = La_xup;

La_c.obj = num;
La_c.con = den;
La_c.tot = La_c.obj + La_c.con;
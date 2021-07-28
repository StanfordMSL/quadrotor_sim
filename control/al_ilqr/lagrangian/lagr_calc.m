function La = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult)

% Unpack some stuff
xs  = lqr.xs;
us  = lqr.us;
T   = lqr.T;


[La_x_o,La_x_c] = lagr_x(X,Xbar,xs,con.cx,mult.lamx,mult.mudx,T);
[La_u_o,La_u_c] = lagr_u(U,Ubar,us,con.cu,mult.lamu,mult.mudu,T);
La_xup = lagr_xup(X,U,Xbar,Ubar,xs,us,T);

La.x_o = La_x_o;
La.x_c = La_x_c;
La.u_o = La_u_o;
La.u_c = La_u_c;
La.xup = La_xup;

La.obj = sum(La_x_o) + sum(La_u_o) + sum(sum(La_xup));
La.con = sum(La_x_c) + sum(La_u_c);
La.tot = La.obj + La.con;

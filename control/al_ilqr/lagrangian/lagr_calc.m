function La = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult)

[La_x_o,La_x_c] = lagr_x(X,Xbar,con.cx,mult.lamx,mult.mudx,lqr);
[La_u_o,La_u_c] = lagr_u(U,Ubar,con.cu,mult.lamu,mult.mudu,lqr);
La_xup = lagr_xup(X,U,Xbar,Ubar,lqr);

La.x_o = La_x_o;
La.x_c = La_x_c;
La.u_o = La_u_o;
La.u_c = La_u_c;
La.xup = La_xup;

La.obj = sum(La_x_o) + sum(La_u_o) + sum(sum(La_xup));
La.con = sum(La_x_c) + sum(La_u_c);
La.tot = La.obj + La.con;

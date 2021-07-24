function [La_obj,La_con,La_tot] = fp_La_upd(xk,uk,xb,ub,xs,us,T,con,lambda,mu_d)

if k < T
    La_obj = dCn(xk,uk,xb,ub,xs,us,1);
else
    La_obj = dCT(xk,uk,xb,ub,xs,us,1);
end

La_con = con_cost(con(:,1),lambda(:,1),mu_d(:,1));

La_tot = La_obj + La_con;

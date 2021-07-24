function con = con_calc(X,U,p_box)

Up = [zeros(4,1) U(:,1:end-1)];

[conx,conx_x,conx_u] = conx_calc(X,p_box);
[conu,conu_x,conu_u] = conu_calc(U,Up);

con.nx = size(conx,1);
con.nu = size(conu,1);

con.c  = [conx ; conu];
con.cx = [conx_x ; conu_x];
con.cu = [conx_u ; conu_u];
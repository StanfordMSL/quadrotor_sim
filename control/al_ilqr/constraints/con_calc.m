function con = con_calc(X,U,p_box)

Up = [zeros(4,1) U(:,1:end-1)];

[con.cx,con.cx_x,con.cx_u] = conx_calc(X,p_box);
[con.cu,con.cu_x,con.cu_u] = conu_calc(U,Up);

end
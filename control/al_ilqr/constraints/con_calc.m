function con = con_calc(obj_type,X,U,p_box,map,cone_coeffs)

Up = [zeros(4,1) U(:,1:end-1)];

n_x = size(X,1);
n_u = size(U,1);
N   = size(X,2);

[con.cx,con.cx_x,con.cx_u] = conx_calc(obj_type,X,p_box,map,cone_coeffs,n_x,n_u,N);
[con.cu,con.cu_x,con.cu_u] = conu_calc(U,Up,n_x,n_u,N);

end
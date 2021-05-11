function lin_func_init(mode,model)

syms FT_ext [6 1] real
syms wt [13 1] real

m = model.m_est;
g = model.g;
D = model.D_est;
I = model.I_est;

switch mode
    case 'direct'
        syms x [13 1] real
        syms u [4 1] real
        
        f = quadcopter_est(x,u,FT_ext,wt);
    case 'u_br'
        syms x [10 1] real
        syms u [4 1] real        
        x_s = [x ; u(2:4)];
        
        syms w_m [4 1] real
        
        f_s = quadcopter_est(x_s,w_m,FT_ext,wt);

        F_g =  m.*[0 ; 0 ; -g];
        F_t =  quatrot2([0 ; 0 ; u(1)],x(7:10));
        F_D = -quatrotmat2(D,x(7:10))*x(4:6);
        v_dot = (1/m) .* ( F_g + F_t + F_D + FT_ext(1:3));

        f_s(4:6) = v_dot;
        
        f = f_s(1:10);
    case 'u_br_pid'
        syms x [19 1] real        
        syms w_m [4 1] real
        
        f_s = quadcopter_est(x(1:13),w_m,FT_ext,wt);

        f
        syms u [4 1] real
        

end

J_x = jacobian(f,x);
J_u = jacobian(f,u);

matlabFunction(J_x,'File','linearization/J_x_calc','vars',{x,u})
matlabFunction(J_u,'File','linearization/J_u_calc','vars',{x,u})
   
disp("[lin_func_init]: Functions Generated")

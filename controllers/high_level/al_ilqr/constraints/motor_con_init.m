function motor_con_init(input_mode,model)

% Unpack some stuff
w_m_min = model.motor.min.*ones(4,1);
w_m_max = model.motor.max.*ones(4,1);

I   = model.est.I;
w2m = model.est.w2m;
kw  = model.est.kw(1,1);
dt  = model.est.dt;

address = 'controllers/high_level/al_ilqr/constraints/';

switch input_mode
    case 'direct'
        syms x [13 1] real
        syms u [4 1] real
                
        con = [u-w_m_max ; -u+w_m_min];
        
        con_x = jacobian(con,x);
        con_u = jacobian(con,u);
        
        matlabFunction(con,'File',[address,'motor_con'],'vars',{x,u})
        matlabFunction(con_x,'File',[address,'motor_con_x'],'vars',{x,u})
        matlabFunction(con_u,'File',[address,'motor_con_u'],'vars',{x,u})
    case 'body_rate'
        syms x [10 1] real
        syms u   [4 1] real
        syms u_p [4 1] real
        
        w = u(2:4);
        w_p = u_p(2:4);
        delta_w = w-w_p;
        
        tau = ((I*delta_w)./dt) + cross(w,I*w);        
        wrench = [u(1) ; tau];
        
        f_m = w2m*wrench;
        w_m = sqrt((1./kw).*f_m);
        con = [w_m-w_m_max ; -w_m+w_m_min];
        
        con_x = jacobian(con,x);
        con_u = jacobian(con,u);

        matlabFunction(con,'File',[address,'motor_con'],'vars',{x,u,u_p})
        matlabFunction(con_x,'File',[address,'motor_con_x'],'vars',{x,u,u_p})
        matlabFunction(con_u,'File',[address,'motor_con_u'],'vars',{x,u,u_p})
end





end
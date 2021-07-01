function motor_con_init(input_mode,model)

% Unpack some stuff
wm_min = model.motor.min.*ones(4,1);
wm_max = model.motor.max.*ones(4,1);

I   = model.est.I;
w2m = model.est.w2m;
kw  = model.est.kw(1,1);
dt  = model.est.dt;

fm_min = kw.*wm_min.^2;
fm_max = kw.*wm_max.^2;

address = 'controllers/high_level/al_ilqr/constraints/';

switch input_mode
    case 'direct'
        syms x [13 1] real
        syms u [4 1] real
            
        fm = kw.*u.^2;       
        con = [fm-fm_max ; -fm+fm_min];
        
        con_x = jacobian(con,x);
        con_u = jacobian(con,u);
        
        matlabFunction(con,'File',[address,'motor_con'],'vars',{x,u})
        matlabFunction(con_x,'File',[address,'motor_con_x'],'vars',{x,u})
        matlabFunction(con_u,'File',[address,'motor_con_u'],'vars',{x,u})
    case 'wrench'
        syms x [13 1] real
        syms u [4 1] real
        
        fm = w2m.*u;
        con = [fm-fm_max ; -fm+fm_min];
     
        con_x = jacobian(con,x);
        con_u = jacobian(con,u);
        
        matlabFunction(con,'File',[address,'motor_con'],'vars',{x,u})
        matlabFunction(con_x,'File',[address,'motor_con_x'],'vars',{x,u})
        matlabFunction(con_u,'File',[address,'motor_con_u'],'vars',{x,u})
    case 'body_rate'
        syms x [10 1] real
        syms u   [4 1] real
        syms up [4 1] real
        
        w = u(2:4);
        wp = up(2:4);
        delta_w = w-wp;
        
        tau = ((I*delta_w)./dt) + cross(w,I*w);        
        wrench = [u(1) ; tau];
        
        fm = w2m*wrench;
        con = [fm-fm_max ; -fm+fm_min];
        
        con_x = jacobian(con,x);
        con_u = jacobian(con,u);

        matlabFunction(con,'File',[address,'motor_con'],'vars',{x,u,up})
        matlabFunction(con_x,'File',[address,'motor_con_x'],'vars',{x,u,up})
        matlabFunction(con_u,'File',[address,'motor_con_u'],'vars',{x,u,up})
end

end
function motor_con_init(model,input_mode)

tic

% Unpack some stuff
wm_min = model.motor.min.*ones(4,1);
wm_max = model.motor.max.*ones(4,1);

I   = model.est.I;
w2m = model.est.w2m;
kw  = model.est.kw(3,1);
dt  = model.est.dt;

fm_min = kw.*wm_min.^2;
fm_max = kw.*wm_max.^2;

address = 'control/al_ilqr/constraints/';

switch input_mode
    case 'direct'
        u = sym('u',[4 1],'real');

        fm = kw.*u.^2;       
        conu = [fm-fm_max ; -fm+fm_min]/fm_max(1);
        
        conu_u = jacobian(conu,u);
        
        matlabFunction(conu,'File',[address,'conu'],'vars',{u});
        matlabFunction(conu_u,'File',[address,'conu_u'],'vars',{u});
    case 'wrench'
        u = sym('u',[4 1],'real');
        
        fm = w2m.*u;
        conu = [fm-fm_max ; -fm+fm_min]/fm_max(1);
     
        conu_u = jacobian(conu,u);
        
        matlabFunction(conu,'File',[address,'conu'],'vars',{u});
        matlabFunction(conu_u,'File',[address,'conu_u'],'vars',{u});
    case 'body_rate'
        u = sym('u',[4 1],'real');
        up = sym('up',[4 1],'real');
        
        w = u(2:4);
        wp = up(2:4);
        delta_w = w-wp;
        
        tau = ((I*delta_w)./dt) + cross(w,I*w);        
        wrench = [fn2f(u(1)) ; tau];
        
        fm = w2m*wrench;
%         conu = [fm-fm_max ; -fm+fm_min]/fm_max(1);
        conu = [fm-fm_max ; -fm+fm_min];

        conu_u = jacobian(conu,u);

        matlabFunction(conu,'File',[address,'conu'],'vars',{u,up});
        matlabFunction(conu_u,'File',[address,'conu_u'],'vars',{u,up});
end

t_comp = toc;

disp(['[motor_con_init]: Motor Constraints Generated in ' num2str(t_comp) 's'])

end
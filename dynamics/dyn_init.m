function model = dyn_init(model,input_mode)

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = sym('p',[3 1],'real');
v = sym('v',[3 1],'real');
q = sym('q',[4 1],'real');
q_c = [q(1) ; -q(2) ; -q(3) ; -q(4)];
w = sym('w',[3 1],'real');

x_s = [p ; v ; q ; w];

% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_b = quatrot2([1 ; 0 ; 0],q);
y_b = quatrot2([0 ; 1 ; 0],q);

v_h = v'*(x_b+y_b);

W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];

% Motor Inputs (rad/s) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

w_m = sym('w_m',[4 1],'real');

% Noise %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wt = sym('wt',[13 1],'real');

% External Forces/Torques %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

F_ext = sym('F_ext',[3 1],'real');
tau_ext = sym('tau_ext',[3 1],'real');

FT_ext = [F_ext ; tau_ext];
    
% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:2
    if k == 1       % Actual
        db = model.act;        
        quad_func = 'dynamics/flight/quadcopter_act';
        w2m_func  = 'dynamics/motor_mapping/w2m_act';
    else            % Estimated
        db = model.est;     
        quad_func = 'dynamics/flight/quadcopter_est';
        w2m_func  = 'dynamics/motor_mapping/w2m_est';
    end
    
    % Unpack
    g  = db.g;
    dt = db.dt;
    kw = db.kw;
    kh = db.kh;
    D = db.D;
    A = db.A;
    B = db.B;
    m = db.m;
    I = db.I;
    m2w = db.m2w;
    w2m = db.w2m;
    
    % Forces
    F_g =  m.*[0 ; 0 ; -g];

    A_w_m = [w_m.^2 w_m 1];
    F_m = sign(w_m).*(A_w_m*kw);
    F_w = m2w*F_m; 
    F_t =  quatrot2([0 ; 0 ; (F_w(1,1) + kh.*v_h.^2)],q);
    
    D_w = quatrotmat2(D,q);
    F_D = -D_w*v;

    % Torques
    tau_mot  =  F_w(2:4,1);
    tau_rot  = -cross(w,I*w);
    tau_linD = -A*quatrot2(v,q_c);
    tau_accD = -B*w;
    
    J = (10^-7).*[ 7.8  0.0  0.0;...
                   0.0  7.8  0.0;...
                   0.0  0.0  9.6];      % Motor Inertia Tensor
    w_mg = [0       0      0      0;...
            0       0      0      0;...
         -w_m(1) -w_m(2) w_m(3) w_m(4)];
    
    tau_g = [0 ; 0 ; 0];
    for k_mg = 1:4
        tau_g = tau_g - J*cross(w,w_mg(:,k_mg));
    end
    
    % Dynamics Equations 
    
    p_dot = v;
    v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
    q_dot = 0.5*W*q;
    w_dot = I\(tau_mot + tau_rot + tau_g + tau_linD + tau_accD + tau_ext);

    x_dot = [p_dot ; v_dot ; q_dot ; w_dot];
    
    % Update Equations 

    x_upd(1:3,1) = p + dt.*p_dot;
    x_upd(4:6,1) = v + dt.*v_dot;  

    q_hat = q + q_dot.*dt;
    x_upd(7:10,1)  = q_hat./norm(q_hat);
    x_upd(11:13,1) = w + dt.*w_dot;

    x_upd = x_upd + wt;

    %% Output Quadcopter Dynamics Function
    matlabFunction(x_upd,'File',quad_func,'vars',{x_s,w_m,FT_ext,wt});
    
    %% Output Wrench to Motor
    wrench = sym('wrench',[4 1],'real');

    mt = w2m*wrench;
    mt_sgn = sign(mt);
    mt_abs = abs(mt);
    
    u_m = mt_sgn.*sqrt(mt_abs./kw(1));
    matlabFunction(u_m,'File',w2m_func,'vars',{wrench});

    %% Output Thrust (f) to Normalized Thrust (fn)
    f2fn_func  = 'dynamics/motor_mapping/f2fn';
    fn2f_func  = 'dynamics/motor_mapping/fn2f';

    f  = sym('f','real');
    fn = sym('fn','real');

    val_min = model.motor.thrust_min;
    val_max = model.motor.thrust_max;
    
    fn_out = (1/4) * ((f-val_min)/(val_max-val_min));
    f_out = 4*fn*(val_max-val_min) + val_min;
    
    matlabFunction(fn_out,'File',f2fn_func,'vars',f);
    matlabFunction(f_out,'File',fn2f_func,'vars',fn);

    % Update the c_hover
    model.motor.c_hover      = f2fn(model.motor.thrust_hover);

    %% Output Linearization Terms
    % Note: we use the estimated model since the linearization is in
    % the controller only.
    if k == 2
        % Linearization for EKF
        x_ekf = x_upd;
        
        x_ses = sym('x_ses',[13 1],'real');
        u_ses = sym('u_ses',[4 1],'real');
        
        x_ekf = subs(x_ekf,p,x_ses(1:3));
        x_ekf = subs(x_ekf,v,x_ses(4:6));
        x_ekf = subs(x_ekf,q,x_ses(7:10));
        x_ekf = subs(x_ekf,w,x_ses(11:13));
               
        x_ekf = subs(x_ekf,w_m,u_ses);

        A_ekf   = jacobian(x_ekf,x_ses);   
        matlabFunction(A_ekf,'File','dynamics/Jacobians/A_ekf_calc','vars',{x_ses,u_ses});
        
        % Linearization for al-iLQR
        switch input_mode
            case 'direct'
                x = sym('x',[13 1],'real');
                u = sym('u',[4 1],'real');
                
                x_opt = x_upd;
                
                x_opt = subs(x_opt,p,x(1:3));
                x_opt = subs(x_opt,v,x(4:6));
                x_opt = subs(x_opt,q,x(7:10));
                x_opt = subs(x_opt,w,x(11:13));
                
                x_opt = subs(x_opt,w_m,u);
           case 'wrench'
                x = sym('x',[13 1],'real');
                u = sym('u',[4 1],'real');
                
                % Forces
                F_g =  m.*[0 ; 0 ; -g];
                F_t =  u(1);
                F_D = -quatrotmat2(D,q)*v;

                % Torques
                tau_mot  =  u(2:4,1);
                tau_rot  = -cross(w,I*w);
                tau_linD = -A*quatrot2(v,q_c);
                tau_accD = -B*w;

                % Dynamics Equations
                p_dot = v;
                v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
                q_dot = 0.5*W*q;
                w_dot = I\(tau_mot + tau_rot + tau_g + tau_linD + tau_accD + tau_ext);
    
                % Update Equations 
                x_opt(1:3,1) = p + dt.*p_dot;
                x_opt(4:6,1) = v + dt.*v_dot;  

                q_hat = q + q_dot.*dt;
                x_opt(7:10,1)  = q_hat./norm(q_hat);
                x_opt(11:13,1) = w + dt.*w_dot;
   
                x_opt = subs(x_opt,p,x(1:3));
                x_opt = subs(x_opt,v,x(4:6));
                x_opt = subs(x_opt,q,x(7:10));
                x_opt = subs(x_opt,w,x(11:13));
                
            case 'body_rate'
                x = sym('x',[10 1],'real');
                u = sym('u',[4 1],'real');
                
                % Forces
                F_g =  m.*[0 ; 0 ; -g];
                F_t =  quatrot2([0 ; 0 ; fn2f(u(1))],q);
                F_D = -quatrotmat2(D,q)*v;
                
                % Dynamic Equations
                p_dot = v;
                v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
                q_dot = 0.5*W*q;

                x_opt(1:3,1) = p + dt.*p_dot;
                x_opt(4:6,1) = v + dt.*v_dot;  

                q_hat = q + dt.*q_dot;
                x_opt(7:10,1)  = q_hat./norm(q_hat);
                
                x_opt = subs(x_opt,p,x(1:3));
                x_opt = subs(x_opt,v,x(4:6));
                x_opt = subs(x_opt,q,x(7:10));
                x_opt = subs(x_opt,w,u(2:4));
        end
        
        A = jacobian(x_opt,x);
        B = jacobian(x_opt,u);
        
        matlabFunction(A,'File','dynamics/Jacobians/A_calc','vars',{x,u});
        matlabFunction(B,'File','dynamics/Jacobians/B_calc','vars',{x,u});
    end

end

disp("[dyn_func_init]: Model Functions Generated")

function dyn_init(model,input_mode)

% Precession (implement later) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau_g = -[0 ; 0 ; 0];

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms p [3 1] real
syms v [3 1] real
syms q [4 1] real
q_c = [q(1) ; -q(2) ; -q(3) ; -q(4)];
syms w [3 1] real

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

syms w_m [4 1] real

% Noise %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms wt [13 1] real

% External Forces/Torques %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms F_ext [3 1] real
syms tau_ext [3 1] real
FT_ext = [F_ext ; tau_ext];
    
% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:2
    if k == 1       % Actual
        db = model.act;        
        quad_func = 'models/flight_dynamics/quadcopter_act';
        w2m_func  = 'models/motor_mapping/w2m_act';
    else            % Estimated
        db = model.est;     
        quad_func = 'models/flight_dynamics/quadcopter_est';
        w2m_func  = 'models/motor_mapping/w2m_est';
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
    F_w = m2w*(kw(1).*w_m.^2 + kw(2).*w_m.^1 + kw(3).*w_m.^0); 

    F_g =  m.*[0 ; 0 ; -g];
    F_t =  quatrot2([0 ; 0 ; (F_w(1,1) + kh.*v_h.^2)],q);
    F_D = -quatrotmat2(D,q)*v;

    % Torques
    tau_mot  =  F_w(2:4,1);
    tau_rot  = -cross(w,I*w);
    tau_linD = -A*quatrot2(v,q_c);
    tau_accD = -B*w;

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
    matlabFunction(x_upd,'File',quad_func,'vars',{x_s,w_m,FT_ext,wt})
    
    %% Output Wrench to Motor
    syms wrench [4 1] real

    T_motor = w2m*wrench;
    u_m = sqrt(T_motor./kw(1));
    matlabFunction(u_m,'File',w2m_func,'vars',{wrench})

    %% Output Linearization Terms
    % Note: we use the estimated model since the linearization is in
    % the controller only.
    if k == 2
        % Linearization for EKF
        x_ekf = x_upd;

        syms x_ses [13 1] real
        syms u_ses [4 1] real
        
        x_ekf = subs(x_ekf,{p1,p2,p3},{x_ses1,x_ses2,x_ses3});
        x_ekf = subs(x_ekf,{v1,v2,v3},{x_ses4,x_ses5,x_ses6});
        x_ekf = subs(x_ekf,{q1,q2,q3,q4},{x_ses7,x_ses8,x_ses9,x_ses10});
        x_ekf = subs(x_ekf,{w1,w2,w3},{x_ses11,x_ses12,x_ses13});

        x_ekf = subs(x_ekf,{w_m1,w_m2,w_m3,w_m4},{u_ses1,u_ses2,u_ses3,u_ses4});

        A_ekf   = jacobian(x_ekf,x_ses);   
        matlabFunction(A_ekf,'File','models/Jacobians/A_ekf_calc','vars',{x_ses,u_ses})
        
        % Linearization for al-iLQR
        switch input_mode
            case 'direct'
                n_x = 13;
                x_opt = x_upd;
                
                syms x [n_x 1] real
                syms u [4 1] real
                
                x_opt = subs(x_opt,{p1,p2,p3},{x1,x2,x3});
                x_opt = subs(x_opt,{v1,v2,v3},{x4,x5,x6});
                x_opt = subs(x_opt,{q1,q2,q3,q4},{x7,x8,x9,x10});
                x_opt = subs(x_opt,{w1,w2,w3},{x11,x12,x13});
                
                x_opt = subs(x_opt,{w_m1,w_m2,w_m3,w_m4},{u1,u2,u3,u4});              
           case 'wrench'
                syms x [13 1] real
                syms u [4 1] real
                
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
   
                x_opt = subs(x_opt,{p1,p2,p3},{x1,x2,x3});
                x_opt = subs(x_opt,{v1,v2,v3},{x4,x5,x6});
                x_opt = subs(x_opt,{q1,q2,q3,q4},{x7,x8,x9,x10});
                x_opt = subs(x_opt,{w1,w2,w3},{x11,x12,x13});
                
            case 'body_rate'
                n_x = 10;
                
                syms x [n_x 1] real
                syms u [4 1] real
                
                % Forces
                F_g =  m.*[0 ; 0 ; -g];
                F_t =  quatrot2([0 ; 0 ; u1],q);
                F_D = -quatrotmat2(D,q)*v;
                
                % Dynamic Equations
                p_dot = v;
                v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
                q_dot = 0.5*W*q;

                x_opt(1:3,1) = p + dt.*p_dot;
                x_opt(4:6,1) = v + dt.*v_dot;  

                q_hat = q + dt.*q_dot;
                x_opt(7:10,1)  = q_hat./norm(q_hat);
                
                x_opt = subs(x_opt,{p1,p2,p3},{x1,x2,x3});
                x_opt = subs(x_opt,{v1,v2,v3},{x4,x5,x6});
                x_opt = subs(x_opt,{q1,q2,q3,q4},{x7,x8,x9,x10});
                x_opt = subs(x_opt,{w1,w2,w3},{u2,u3,u4});
        end
        
        A = jacobian(x_opt,x);
        B = jacobian(x_opt,u);
        
        matlabFunction(A,'File','models/Jacobians/A_calc','vars',{x,u})
        matlabFunction(B,'File','models/Jacobians/B_calc','vars',{x,u})
    end

end

disp("[dyn_func_init]: Model Functions Generated")

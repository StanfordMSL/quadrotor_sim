function quad2Dsym(model)

    nx = 6;
    nu = 2;
    
    x = sym('x',[nx 1],'real');
    u = sym('u',[nu 1],'real');
    
    th  =  x(3);
    v   = [x(4) ; x(5)];   % Linear Velocity
    thd =  x(6);            % Angular Velocity

    % Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Unpack
    dt    = model.est.dt;
    g     = model.est.g;
    m     = model.est.m;
    Iqd   = model.est.I(2,2);

    % Forces
    F_g   = m.*[0 ; -g];                        % Gravity
    F_t   = [-u(1)*sin(th) ; u(1)*cos(th)];     % Thrust

    % Torques
    tau_wr   = u(2);                            % Torque Input

    % Dynamics Equations
    p_dot   = v;
    th_dot  = thd;
    v_dot   = (1/m) .* ( F_g + F_t);
    thd_dot = tau_wr/Iqd;

    x_dot = [ p_dot ; th_dot ; v_dot ; thd_dot];
    
    x_upd = x + dt.*x_dot;
    x_upd = simplify(x_upd);
    
    A = jacobian(x_upd,x);
    B = jacobian(x_upd,u);
    
    H1 = hessian(x_upd(1),x);
    H2 = hessian(x_upd(2),x);
    H3 = hessian(x_upd(3),x);
    H4 = hessian(x_upd(4),x);
    H5 = hessian(x_upd(5),x);
    H6 = hessian(x_upd(6),x);
    
    matlabFunction(A,'File','LQR/A_calc','vars',{x,u});
    matlabFunction(B,'File','LQR/B_calc','vars',{x,u});  
    
    matlabFunction(H1,'File','LQR/H1_calc','vars',{x,u});
    matlabFunction(H2,'File','LQR/H2_calc','vars',{x,u});  
    matlabFunction(H3,'File','LQR/H3_calc','vars',{x,u});
    matlabFunction(H4,'File','LQR/H4_calc','vars',{x,u});  
    matlabFunction(H5,'File','LQR/H5_calc','vars',{x,u});
    matlabFunction(H6,'File','LQR/H6_calc','vars',{x,u});  
end
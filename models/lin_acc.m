function vel_dot = lin_acc(states,u,model,F_ext,frame,mode)
   % q0 = sqrt(1-states(7:9,1)'*states(7:9,1));
   % quat = [q0 ; states(7:9,1)];
    quat = states(7:10);
    k2 = model.kt_act(1,1);
    k1 = model.kt_act(2,1);
    k0 = model.kt_act(3,1);
    
    % Type Here Refers to Real = 0, and Model = 1.        
    f = zeros(4,1);
    switch mode
        case 'actual'
            for k = 1:4
                f(k,1) = k2*(u(k,1)^2)+k1*u(k,1)+k0;
            end
            F_drag = -model.kd_act .* states(4:6,1);
            m = model.m_act;
        case 'estimate'
            for k = 1:4
                f(k,1) = model.k2_est(1,1)*(u(k,1)^2);
            end 
            F_drag = zeros(3,1);
            m = model.m_est;
    end
    
    F_grav   = [ 0 ; 0 ; -m*model.g];
    F_thrust = [ 0 ; 0 ; sum(f)];

    bRw = quat2rotm(quat');
    wRb = bRw';
    vel_dot = (1/m)*(F_grav + bRw*F_thrust + F_drag + F_ext);  % default is world frame
    
    if frame == 1 % body
        vel_dot = wRb*vel_dot;
    end
end
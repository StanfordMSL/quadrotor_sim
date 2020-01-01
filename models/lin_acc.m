function vel_dot = lin_acc(states,m,f,F_drag,F_ext,model,frame)
    q0 = sqrt(1-states(7:9,1)'*states(7:9,1));
    quat = [q0 ; states(7:9,1)];  
    
    F_grav   = [ 0 ; 0 ; -m*model.g];
    F_thrust = [ 0 ; 0 ; sum(f)];

    bRw = quat2rotm(quat');
    wRb = bRw';
    vel_dot = (1/m)*(F_grav + bRw*F_thrust + F_drag + F_ext);  % default is world frame
    
    if frame == 1 % body
        vel_dot = wRb*vel_dot;
    end
end
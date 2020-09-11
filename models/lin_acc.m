function vel_dot = lin_acc(states,m,f,F_drag,F_ext,g,frame)
    quat = states(7:10,1);  
    
    F_grav   = [ 0 ; 0 ; -m*g];
    F_thrust = [ 0 ; 0 ; sum(f)];

    bRw = quat2rotm(quat');
    wRb = bRw';
    vel_dot = (1/m)*(F_grav + bRw*F_thrust + F_drag + F_ext);  % default is world frame
    
    if frame == 1 % body
        vel_dot = wRb*vel_dot;
    end
end
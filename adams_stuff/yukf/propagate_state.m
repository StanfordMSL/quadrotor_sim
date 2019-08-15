function est_next_state = propagate_state(state, model, u)
    est_next_state = zeros(size(state));
    
    % Predict Positions
    est_next_state(1:3) = state(1:3) + model.con_dt * state(4:6);

    % Predict Velocities
    if(~isempty(u))
        vel_dot = lin_acc(state, u, model,[0 0 0]', 0, 'actual');
        est_next_state(4:6) = state(4:6) + model.con_dt * vel_dot;
    else
        est_next_state(4:6) = state(4:6); 
    end

    % Predict Quaternions
    % omegas
    wx = state(10);
    wy = state(11);
    wz = state(12);

    % Setup Some Useful Stuff for Pred 
    Omega = [ 0 -wx -wy -wz ;...
             wx   0  wz -wy ;...
             wy -wz   0  wx ;...
             wz  wy -wx   0 ];

    quat = complete_unit_quat(state(7:9));
    q_hat = quat + 0.5 * Omega * quat * model.con_dt;
    est_next_state(7:9) = q_hat(2:4);

    % Predict Angular Velocities
    if(~isempty(u))
        omega_dot = ang_acc(u, state(10:12), model, [0 0 0]', 'actual');
        est_next_state(10:12) = state(10:12) + omega_dot * model.con_dt;
    else
        est_next_state(10:12) = state(10:12);
    end
    
end

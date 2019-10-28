function est_next_state = step_quad_dynamics(one_obj_state, model, u, dt)
    global yukf
    est_next_state = zeros(size(one_obj_state));
    
    % Predict Positions
    est_next_state(1:3) = one_obj_state(1:3) + dt * one_obj_state(4:6);

    % Predict Velocities
    if(~isempty(u))
        vel_dot = lin_acc(one_obj_state, u, model,[0 0 0]', 0, 'actual');
        est_next_state(4:6) = one_obj_state(4:6) + dt * vel_dot;
    else
        est_next_state(4:6) = one_obj_state(4:6); 
    end

    % Predict Quaternions
    quat = one_obj_state(7:10);
    
    b_use_paper_method = true;
    if b_use_paper_method
        w_vec = one_obj_state(11:13);
        % convert omega to angle axis
        nrm = norm(w_vec);
        ang = nrm * dt;
        ax = w_vec / nrm;
        quat_delta = axang_to_quat(ax*ang);
        q_hat = quatmultiply(quat_delta(:)', quat(:)')';
        disp('')
    else
        % omegas
        wx = one_obj_state(11);
        wy = one_obj_state(12);
        wz = one_obj_state(13);

        % Setup Some Useful Stuff for Pred 
        Omega = [ 0 -wx -wy -wz ;...
                 wx   0  wz -wy ;...
                 wy -wz   0  wx ;...
                 wz  wy -wx   0 ];

        q_hat = quat + 0.5 * Omega * quat * dt;
    end
    disp('')
    if isempty(u) % this is true for the tracked quad
        if any([yukf.prms.b_enforce_0_yaw, yukf.prms.b_enforce_yaw, yukf.prms.b_enforce_pitch, yukf.prms.b_enforce_roll])
            q_hat = cheat_with_angles(q_hat);
        end
    end
    disp('')
    q_hat = normalize_quat(q_hat);
    est_next_state(7:10) = q_hat;

    % Predict Angular Velocities
    if(~isempty(u))
        omega_dot = ang_acc(u, one_obj_state(11:13), model, [0 0 0]', 'actual');
        est_next_state(11:13) = one_obj_state(11:13) + dt * omega_dot;
    else
        est_next_state(11:13) = one_obj_state(11:13);
    end
end
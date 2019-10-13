function est_next_state = propagate_state(state, model, u, dt)
    global yukf flight k
    est_next_state = zeros(size(state));
    
    % Predict Positions
    est_next_state(1:3) = state(1:3) + dt * state(4:6);

    % Predict Velocities
    if(~isempty(u))
        vel_dot = lin_acc(state', u, model,[0 0 0]', 0, 'actual');
        est_next_state(4:6) = state(4:6) + dt * vel_dot';
    else
        est_next_state(4:6) = state(4:6); 
    end

    % Predict Quaternions
    quat =state(7:10);
    
    b_use_paper_method = true;
    if b_use_paper_method
        w_vec = state(11:13);
        % convert omega to angle axis
        nrm = norm(w_vec);
        ang = nrm * dt;
        ax = w_vec / nrm;
        quat_delta = axang_to_quat(ax*ang);
%         quat_delta = axang_to_quat(w_vec);
%         q_hat = quatmultiply(quat(:)', quat_delta(:)')';
        q_hat = quatmultiply(quat_delta(:)', quat(:)')';
        disp('')
    else
        % omegas
        wx = state(11);
        wy = state(12);
        wz = state(13);

        % Setup Some Useful Stuff for Pred 
        Omega = [ 0 -wx -wy -wz ;...
                 wx   0  wz -wy ;...
                 wy -wz   0  wx ;...
                 wz  wy -wx   0 ];

        q_hat = quat + 0.5 * Omega * quat * dt;
    end
    disp('')
%     if any([yukf.prms.b_enforce_0_yaw, yukf.prms.b_enforce_yaw, yukf.prms.b_enforce_pitch, yukf.prms.b_enforce_roll])
%         q_hat = cheat_with_angles(q_hat);
%     end
    disp('')
    q_hat = normalize_quat(q_hat);
    est_next_state(7:10) = q_hat;
%     vec_norm = norm( est_next_state(7:9) );
%     if( vec_norm > 1 && vec_norm < 1.001 )
%         % just a numerical thing
%         est_next_state(7:9) = est_next_state(7:9) / vec_norm;
%     end

    % Predict Angular Velocities
    if(~isempty(u))
        omega_dot = ang_acc(u, state(11:13), model, [0 0 0]', 'actual');
        est_next_state(11:13) = state(11:13) + dt * omega_dot';
    else
        est_next_state(11:13) = state(11:13);
    end
    
end

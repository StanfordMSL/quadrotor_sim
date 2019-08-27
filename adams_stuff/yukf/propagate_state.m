function est_next_state = propagate_state(state, model, u, dt)
    global yukf
    est_next_state = zeros(size(state));
    
    % Predict Positions
    est_next_state(1:3) = state(1:3) + dt * state(4:6);

    % Predict Velocities
    if(~isempty(u))
        vel_dot = lin_acc(state, u, model,[0 0 0]', 0, 'actual');
        est_next_state(4:6) = state(4:6) + dt * vel_dot;
    else
        est_next_state(4:6) = state(4:6); 
    end

    % Predict Quaternions
    quat = complete_unit_quat(state(7:9));
    
    b_use_paper_method = false;
    if b_use_paper_method
        w_vec = state(10:12);
        % convert omega to angle axis
%         nrm = norm(w_vec);
%         ang = nrm * model.con_dt;
%         ax =  w_vec / nrm;
%         quat_delta = axang_to_quat(ax*ang);
        quat_delta = axang_to_quat(w_vec);
        q_hat = quatmultiply(quat(:)', quat_delta(:)')';
    else
        % omegas
        wx = state(10);
        wy = state(11);
        wz = state(12);

        % Setup Some Useful Stuff for Pred 
        Omega = [ 0 -wx -wy -wz ;...
                 wx   0  wz -wy ;...
                 wy -wz   0  wx ;...
                 wz  wy -wx   0 ];

        q_hat = quat + 0.5 * Omega * quat * dt;
        % zero yaw
        if yukf.prms.b_enforce_0_yaw
            [~, pitch, roll] = quat2angle(q_hat(:)');
            q_hat = angle2quat(0, pitch, roll)';
        end
        %%%%%%%%%%%%%%
        if( norm(q_hat(2:4)) <= 1 )
            q_hat = complete_unit_quat(q_hat(2:4)); % normalize quaternion    
        elseif( norm(q_hat(2:4)) <= 1.001 )
            q_hat = q_hat / norm(q_hat); % no warning needed - close enough
        else
            warning("invalid quaternion - crude normalization")
            q_hat = q_hat / norm(q_hat);
        end
    end
    est_next_state(7:9) = q_hat(2:4);
    vec_norm = norm( est_next_state(7:9) );
    if( vec_norm > 1 && vec_norm < 1.001 )
        % just a numerical thing
        est_next_state(7:9) = est_next_state(7:9) / vec_norm;
    end

    % Predict Angular Velocities
    if(~isempty(u))
        omega_dot = ang_acc(u, state(10:12), model, [0 0 0]', 'actual');
        est_next_state(10:12) = state(10:12) + dt * omega_dot;
    else
        est_next_state(10:12) = state(10:12);
    end
    
end

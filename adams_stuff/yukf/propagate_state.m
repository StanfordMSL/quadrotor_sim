function est_next_state = propagate_state(state, model, u)
    est_next_state = zeros(size(state));
    
    % Predict Positions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    est_next_state(1:3) = state(1:3) + model.con_dt * state(4:6);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Predict Velocities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(~isempty(u))
        vel_dot = lin_acc(state, u, model,[0 0 0]', 0, 'actual');
        est_next_state(4:6) = state(4:6) + model.con_dt * vel_dot;
    else
        est_next_state(4:6) = state(4:6); 
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Predict Angular Velocities %%%%%%%%%%%%%%%%%%%%%%%%
    if(~isempty(u))
        omega_dot = ang_acc(u, state(7:9), model, [0 0 0]', 'actual');
        est_next_state(7:9) = state(7:9) + omega_dot * model.con_dt;
    else
        est_next_state(7:9) = state(7:9);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Predict Quaternions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quat = state(10:13);
    
    b_use_paper_method = true;
    if b_use_paper_method
        w_vec = state(7:9);
        % convert omega to angle axis
%         nrm = norm(w_vec);
%         ang = nrm * model.con_dt;
%         ax =  w_vec / nrm;
%         quat_delta = axang_to_quat(ax*ang);
        quat_delta = axang_to_quat(w_vec);
        q_hat = quatmultiply(quat(:)', quat_delta(:)')';
    else
        % omegas
        wx = state(7);
        wy = state(8);
        wz = state(9);

        % Setup Some Useful Stuff for Pred 
        Omega = [ 0 -wx -wy -wz ;...
                 wx   0  wz -wy ;...
                 wy -wz   0  wx ;...
                 wz  wy -wx   0 ];

        q_hat = quat + 0.5 * Omega * quat * model.con_dt;
    end
    est_next_state(10:13) = q_hat;
    vec_norm = norm( est_next_state(11:13) );
    if( vec_norm < 1 )
        % We have a valid quaternion, normalize to make sure norm is 1
        est_next_state(10) = sqrt(1 - vec_norm^2);
    elseif( vec_norm > 1 && vec_norm < 1.001 )
        % a small numeric issue, try to fix
        est_next_state(10) = 0;
        est_next_state(11:13) = est_next_state(11:13) / vec_norm;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
end

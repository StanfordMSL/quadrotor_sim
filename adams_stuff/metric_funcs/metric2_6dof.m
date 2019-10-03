function [prct_crt, err_pos_vec, err_ang_vec] = metric2_6dof(states_est, states_gt)
    % separate position & angular errors
    n = size(states_est, 2);
    thresh_ang  = 5 * pi / 180; % [degrees]
    thresh_pos  = 0.05; % [m]
    
    err_ang_vec = zeros(n, 1);
    err_pos_vec = sum((states_est(1:3, :) - states_gt(1:3, :)).^2, 1).^0.5';
    for state_ind = 1:n
            q_est = states_est(7:10, state_ind); 
            q_gt = states_gt(7:10, state_ind); 
            err_ang_vec(state_ind) = quat_dist(q_gt, q_est);
    end
    prct_crt = 100 * sum(err_pos_vec <= thresh_pos & err_ang_vec <= thresh_ang) / n;
end

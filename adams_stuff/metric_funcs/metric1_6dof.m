function [prct_crt_1, prct_crt_5, prct_crt_10, err_vec, err_ave, err_min, err_max] = metric1_6dof(states_est, states_gt, bb_quad)
    % err = average bounding box vert dist err, % crt based on % of thsee <= thresh
    n = size(states_est, 2);
    num_verts = size(bb_quad, 1);
    bb_diam = norm(bb_quad(1, :) - bb_quad(7, :)); % diameter of bounding box (front left up minus back right down)
    thresh_1  = 0.01 * bb_diam; % 1 % of bb diameter
    thresh_5  = 0.05 * bb_diam; % 5 % of bb diameter
    thresh_10 = 0.10 * bb_diam; % 10 % of bb diameter
    err_vec = zeros(n, 1);
    for state_ind = 1:n
        %  get estimated bounding box (world frame)
        tf_w_quad_est = state_to_tf(states_est(:, state_ind));
        bb_w_est = (tf_w_quad_est * [bb_quad, ones(num_verts, 1)]')';
        bb_w_est = bb_w_est(:, 1:3);
        
        %  get ground truth bounding box  (world frame)
        tf_w_quad_gt = state_to_tf(states_gt(:, state_ind));        
        bb_w_gt = (tf_w_quad_gt * [bb_quad, ones(num_verts, 1)]')';
        bb_w_gt = bb_w_gt(:, 1:3);
        
        err_vec(state_ind) = mean(sum((bb_w_est - bb_w_gt).^2, 2).^0.5);
    end
    prct_crt_1 = 100 * sum(err_vec <= thresh_1) / n;
    prct_crt_5 = 100 * sum(err_vec <= thresh_5) / n;
    prct_crt_10 = 100 * sum(err_vec <= thresh_10) / n;
    err_ave = mean(err_vec);
    err_min = min(err_vec);
    err_max = max(err_vec);
end

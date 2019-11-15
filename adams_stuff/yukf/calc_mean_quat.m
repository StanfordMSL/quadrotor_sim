function [mu_bar, ei_vec_set] = calc_mean_quat(sps, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf, sec 3.4
    max_itr = 50;
    thresh = 0.05 * pi/180;
    
    % calculate the mean of the vector parts
    mu_bar = (yukf.w0_m * sps(:,1)) + (yukf.wi*sum(sps(:,2:end), 2));
    
    % now deal with the quaterion...
    num_sps = size(sps, 2);
    q_bar = sps(7:10, 1);
    q_bar_inv = quatinv(q_bar(:)');
    ei_vec_set = zeros(3, num_sps);
    quat_arr = zeros(num_sps, 4);
    
    % construct weight vector for alternate quat-mean calc
    ws = ones(size(quat_arr, 1), 1) * yukf.wi;
    ws(1) = yukf.w0_m;
    for itr = 1:max_itr
        W = yukf.w0_m;
        e_vec = zeros(3,1);
        for sp_ind = 1:num_sps
            qi = sps(7:10, sp_ind); % quaternion from sigma point
            quat_arr(sp_ind, :) = qi(:)';
            ei_quat = quatmultiply(qi(:)', q_bar_inv(:)'); % quaternion differnce between sigma point and our current estimate of average quat
            ei_vec = quat_to_axang(ei_quat(:)'); % that difference, now in ax ang form
            ei_vec_set(:, sp_ind) = ei_vec; % storing ax ang difference for later use
            e_vec = e_vec(:) + W * ei_vec(:); % updating our loss
            if(sp_ind == 1); W = yukf.wi; end % after first iter, weight changes to wi from w0
        end
        e_quat = axang_to_quat(e_vec);
        q_bar = quatmultiply(e_quat(:)', q_bar(:)');
        q_bar_inv = quatinv( q_bar(:)' );
        
        quatAverage = wavg_quaternion_markley(quat_arr, ws)';
        if abs(quat_dist(quatAverage, q_bar)) > 0.5 * 180/pi
            warning('Built-in Function for quat mean disagrees with us!')
        end
        if(norm(e_vec) < thresh)
            if(itr == 1); continue; end % so quick its worth taking another step
            mu_bar(7:10) = q_bar;
            return  % we have converged
        end
    end
    warning('orientation mean calculation did not converge, using alt mean-quat method! (%.5f)', norm(e_vec))
    mu_bar(7:10) = quatAverage;
end
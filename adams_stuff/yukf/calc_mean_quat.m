function [quat_mean, ei_vec_set] = calc_mean_quat(quat_arr, yukf)
    num_quats = size(quat_arr, 1);
    ei_vec_set = zeros(3, num_quats);
    
    % construct weights used for alternate quat-mean calc
    ws = ones(num_quats, 1) * yukf.wi;
    ws(1) = yukf.w0_m;
    
    
    % TEMP - only use simple method %%%%%%%%%%%%%%%
    q_bar = wavg_quaternion_markley(quat_arr, ws)';
    if q_bar(1) < 0
        q_bar = q_bar * sign(q_bar(1));
    end
    q_bar_inv = quatinv(q_bar(:)');
    for i = 1:num_quats
        qi = quat_arr(i, :);
        ei_quat = quatmultiply(qi(:)', q_bar_inv(:)');
        ei_vec = quat_to_axang(ei_quat(:)');
        ei_vec_set(:, i) = ei_vec;
    end
    quat_mean = q_bar;
    return;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf, sec 3.4
    max_itr = 50;
    thresh = 0.05 * pi/180;
    
    
    % now deal with the quaterion...
    num_sps = size(sps, 2);
    q_bar = sps(7:10, 1);
    q_bar_inv = quatinv(q_bar(:)');
    quat_arr = zeros(num_sps, 4);
    
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
        q_dist = abs(quat_dist(quatAverage, q_bar)); %  THIS IS IN DEGREES
        if q_dist > 0.5 
            warning('Built-in Function for quat mean disagrees with us!(%.5f deg)', q_dist)
        end
        if(norm(e_vec) < thresh)
            if(itr == 1); continue; end % so quick its worth taking another step
            mu_bar(7:10) = q_bar;
            return  % we have converged
        end
    end
    warning('orientation mean calculation did not converge, using alt mean-quat method! (%.5f deg)', norm(e_vec)*180/pi)
    mu_bar(7:10) = quatAverage;
end
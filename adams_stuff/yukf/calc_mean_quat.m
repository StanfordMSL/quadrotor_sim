function [mu_bar, ei_vec_set] = calc_mean_quat(sps, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf, sec 3.4
    max_itr = 40;
    thresh = 0.05 * pi/180;
    
    % calculate the mean of the vector parts
    mu_bar = (yukf.w0_m * sps(:,1)) + (yukf.wi*sum(sps(:,2:end), 2));
    
    
    % now deal with the quaterion...
    num_sps = size(sps, 2);
    q_bar = complete_unit_quat(sps(7:9, 1));
    q_bar_inv = quatinv(q_bar(:)');
%     ei_quat_set = zeros(4, num_sps);
    ei_vec_set = zeros(3, num_sps);
    for itr = 1:max_itr
        W = yukf.w0_m;
        e_vec = zeros(3,1);
        for sp_ind = 1:num_sps
            qi = complete_unit_quat(sps(7:9, sp_ind)); % quaternion from sigma point
            ei_quat = quatmultiply(qi(:)' , q_bar_inv(:)'); % quaternion differnce between sigma point and our current estimate of average quat
            ei_vec = quat_to_axang(ei_quat(:)'); % that difference, now in ax ang form
            ei_vec_set(:, sp_ind) = ei_vec; % storing ax ang difference for later use
            e_vec = e_vec(:) + W * ei_vec(:); % updating our loss
%             qi = complete_unit_quat(sps(7:9, sp_ind));
%             ei_quat_set(:, sp_ind) = quatmultiply(qi(:)' , q_bar_inv(:)');
%             ei_vec = quat_to_axang(ei_quat_set(:, sp_ind)')';
%             e_vec = e_vec(:) + W * ei_vec(:);
            if(sp_ind == 1); W = yukf.wi; end % after first iter, weight changes to wi from w0
        end
        e_quat = axang_to_quat(e_vec);
        q_bar = quatmultiply(e_quat(:)', q_bar(:)');
        q_bar_inv = quatinv( q_bar(:)' );
        
        if(norm(e_vec) < thresh)
            if(itr == 1); continue; end % so quick its worth taking another step
            % we have converged
            mu_bar(7:9) = q_bar(2:4);
            return
        end
    end
    warning('orientation mean calculation did not converge!')
    mu_bar(7:9) = q_bar(2:4);
end
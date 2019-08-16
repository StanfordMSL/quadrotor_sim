function [mu_bar, ei_vec_set] = calc_mean_quat(sps, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf, sec 3.4
    max_itr = 20;
    thresh = 0.1 * pi/180;
    
    % calculate the mean of the vector parts
    mu_bar = (yukf.w0_m * sps(:,1)) + (yukf.wi*sum(sps(:, 2:end), 2));
    
    % now deal with the quaterion...
    num_sps = size(sps, 2);
    q_bar = sps(10:13, 1);
    q_bar_inv = quatinv(q_bar(:)');
    ei_vec_set = zeros(3, num_sps);
    for itr = 1:max_itr
        W = yukf.w0_m;
        e_vec = zeros(3,1);
        for sp_ind = 1:num_sps
            qi = sps(10:13, sp_ind);
            ei_quat = quatmultiply(qi(:)' , q_bar_inv(:)');
            ei_vec_set(:, sp_ind) = quat_to_axang(ei_quat(:)')';
            e_vec = e_vec(:) + W * ei_vec_set(:, sp_ind);
%             warning("might need to wrap to pi here!!")
            if(sp_ind == 1); W = yukf.wi; end % after first iter, weight changes to wi from w0
        end
        e_quat = axang_to_quat(e_vec);
        q_bar = quatmultiply(e_quat(:)', q_bar(:)');
        q_bar_inv = quatinv( q_bar(:)' );
        
        if(norm(e_vec) < thresh)
            if(itr == 1); continue; end % so quick its worth taking another step
            % we have converged
            mu_bar(10:13) = q_bar;
            return
        end
    end
    warning('orientation mean calculation did not converge!')
    mu_bar(10:13) = q_bar;
end
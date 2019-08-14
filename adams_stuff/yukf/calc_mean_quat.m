function q_bar = calc_mean_quat(sps, w0_m, wi)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf, sec 3.4
    max_itr = 20;
    thresh = 0.5 * pi/180;
    
    num_sps = size(sps, 2);
    q_bar = complete_unit_quat(sps(7:9, 1));
    q_bar_inv = quatinv(q_bar(:)');
    for itr = 1:max_itr
        W = w0_m;
        e_vec = zeros(3,1);
        for sp_ind = 1:num_sps
            qi = complete_unit_quat(sps(7:9, sp_ind));
            ei_quat = quatmultiply(qi(:)' , q_bar_inv(:)');
            ei_vec = quat_to_axang(ei_quat(:)');
            e_vec = e_vec(:) + W * ei_vec(:);
            if(sp_ind == 1); W = wi; end % after first iter, weight changes to wi from w0
        end
        e_quat = axang_to_quat(e_vec);
        q_bar = quatmultiply(e_quat(:)', q_bar(:)');
        q_bar_inv = quatinv( q_bar(:)' );
        
        if(norm(e_quat(2:4)) < thresh)
            % we have converged
            break;
        end
    end
end
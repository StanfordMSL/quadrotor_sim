function output = get_angled_bounding_box(bb_rc_list, x_curr, b_draw_box, camera)
    b_use_matlab_angled_bb_func = true;
    if isnan(bb_rc_list(1,1))
        output = [0;0;0;0;0];
    elseif b_use_matlab_angled_bb_func
        min_rect = minBoundingBox(bb_rc_list');
        y1 = min_rect(1,1);
        y2 = min_rect(1,2);
        y3 = min_rect(1,3);
        y4 = min_rect(1,4);
        x1 = min_rect(2,1);
        x2 = min_rect(2,2);
        x3 = min_rect(2,3);
        x4 = min_rect(2,4);
        output = bb_corners_to_angle(x1,y1,x2,y2,x3,y3,x4,y4);
    else
        % calculate the angle of the bounding box - modify the angles to be 
        %   between +- 90 deg since the neural net wont be able to tell the 
        %   difference then keep the angle keeping the bounding box most horizontal

        pairs = [1, 2; 2, 3; 1, 5]; % three vertex pairs that form perp. edges
        angs = nan(3, 1);
        signs = angs;
        output = Inf;
        min_bb_area = Inf;
        for p_ind = 1:size(pairs, 1)
            ind1 = pairs(p_ind, 1);
            ind2 = pairs(p_ind, 2);
            if bb_rc_list(ind1, 2) > bb_rc_list(ind2, 2)
                ur = bb_rc_list(ind1, :);
                ul = bb_rc_list(ind2, :);
            else
                ur = bb_rc_list(ind2, :);
                ul = bb_rc_list(ind1, :);
            end
            angs(p_ind) = wrapToPi(atan2(ur(1) - ul(1), ur(2) - ul(2))); 
            signs(p_ind) = sign(angs(p_ind));
            angs(p_ind) = abs(angs(p_ind));
            angs(p_ind) = min(angs(p_ind), pi - angs(p_ind));

            % find the extent of the bounding box along the angled axes
            box_ang = signs(p_ind)*angs(p_ind);
            rot_mat = [ cos(box_ang), -sin(box_ang);
                        sin(box_ang),  cos(box_ang) ];
            pix_mean = mean(bb_rc_list);
            ax_aligned_pix = (rot_mat * (bb_rc_list - pix_mean)')';
            width_pixel = max(ax_aligned_pix(:, 2)) - min(ax_aligned_pix(:, 2));
            height_pixel = max(ax_aligned_pix(:, 1)) - min(ax_aligned_pix(:, 1));
            bb_area = width_pixel*height_pixel;
            if bb_area <= min_bb_area && abs(box_ang) < output(end)
                min_bb_area = bb_area;
                max_rc = max(bb_rc_list);
                min_rc = min(bb_rc_list);
                center_rc = (max_rc + min_rc)/2;
                output = [center_rc(:); width_pixel; height_pixel; box_ang];
            end

        end
        if b_draw_box
            plot_bounding_angled_box(center_rc, width_pixel, height_pixel, box_ang, bb_rc_list, x_curr(1:3, 1), x_curr(7:10, 1), camera);
        end
    end
end

    
    
%     quad_aligned_bb = [l/2, w/2, h/2;... %    1 front, left, up (from quad's perspective)
%                        l/2, -w/2, h/2;... %   2 front, right, up
%                        -l/2, -w/2, h/2;... %  3 back, right, up
%                        -l/2, w/2, h/2; ... %  4 back, left, up
%                        l/2, w/2, -h/2;... %   5 front, left, down
%                        l/2, -w/2, -h/2;... %  6 front, right, down
%                        -l/2, -w/2, -h/2;... % 7 back, right, down
%                        -l/2, w/2, -h/2 ]; %   8 back, left, down
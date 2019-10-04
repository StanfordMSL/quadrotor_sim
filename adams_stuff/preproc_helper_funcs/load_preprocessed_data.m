function [t_pose_arr, t_rbg_arr, z_mat, position_mat, quat_mat, gt_bb] = load_preprocessed_data_backup(data_dir, yukf, conf_thresh)
    % load in data specified by the date/time string
    
    if yukf.prms.b_angled_bounding_box
        fileID = fopen(sprintf('%s/results_%s_angle.txt', data_dir, yukf.hdwr_prms.datetime_str));
        C = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f');  % name, conyukf.f, min_x, min_y, max_x, max_y
    else
        fileID = fopen(sprintf('%s/results_%s.txt', data_dir, yukf.hdwr_prms.datetime_str));
        C = textscan(fileID,'%s %f %f %f %f %f');  % name, conyukf.f, min_x, min_y, max_x, max_y
    end
    fclose(fileID);
    num_yolo_outputs = length(C{1}); % NOTE: Can be multple per image!
    
    % variables to hold loaded data    
    t_pose_arr = zeros(1, num_yolo_outputs);
    t_rbg_arr = zeros(1, num_yolo_outputs);
    meas_dim = length(yukf.prms.R);
    z_mat = zeros(num_yolo_outputs, meas_dim);
    position_mat = zeros(num_yolo_outputs, 3);
    quat_mat = zeros(num_yolo_outputs, 4);
    gt_bb = zeros(num_yolo_outputs, 4);
    kept_frame_ids = false(1, num_yolo_outputs);
    
    % load the data
    frame_prefix_len = -1;  % # of characters before index (e.g. rbg_03.png --> length(rbg_) = 4)
    last_frame_ind = -1;
    last_conf = -1;
    for ind = 1:num_yolo_outputs
        frame_name = C{1}{ind};
        if frame_prefix_len < 0
            tokens = strsplit(frame_name, '_');
            frame_prefix_len = length(tokens{1}) + 1;
        end
        frame_ind = str2double(frame_name(frame_prefix_len + 1:end));        
        if yukf.hdwr_prms.end_img_ind > 0 && frame_ind > yukf.hdwr_prms.end_img_ind
            break;
        end
        
        conf = C{2}(ind);
        if last_frame_ind == frame_ind && conf < last_conf
            continue; % keep the most confident frame
        end

        if yukf.prms.b_angled_bounding_box
            z_mat(ind,:) = bb_corners_to_angle(C{3}(ind),C{4}(ind),C{5}(ind),C{6}(ind),C{7}(ind),C{8}(ind),C{9}(ind),C{10}(ind));
        else
            x_min = C{3}(ind); % x is col
            y_min = C{4}(ind); % y is row
            x_max = C{5}(ind);
            y_max = C{6}(ind);
            z_mat(ind, :) = [(y_max + y_min)/2, (x_max + x_min)/2, x_max - x_min, y_max - y_min,]; % [r_center, c_center, width, height]

        end
        
        fileID2 = fopen(sprintf('%s/pose_gtboxes_and_time/pose_gtboxes_and_time_%d.txt', data_dir, frame_ind));
        D = textscan(fileID2,'%f %f %f %f %f %f %f %f %f %f %f %f %f'); % x, y, z, qw, qx, qy, qz, max_coords (c/x_max, r/y_max), min_coords (c/x_min, r/y_min)
        fclose(fileID2);
        
        t_rbg_arr(frame_ind + 1) = D{1}(1);
        t_pose_arr(frame_ind + 1) = D{2}(1);
        position_mat(frame_ind + 1, :) = [D{3}(1), D{4}(1), D{5}(1)];
        quat_mat(frame_ind + 1, :) = [D{6}(1), D{7}(1), D{8}(1), D{9}(1)];
        gt_bb(frame_ind + 1, :) = max_min_coords_to_yolo_bb([D{10}(1), D{11}(1)], [D{12}(1), D{13}(1)]);
    end
    t_pose_arr(frame_ind + 1:end) = [];
    t_rbg_arr(frame_ind + 1:end) = [];
    z_mat(frame_ind + 1:end, :) = [];
    position_mat(frame_ind + 1:end, :) = [];
    quat_mat(frame_ind + 1:end, :) = [];
    gt_bb(frame_ind + 1:end, :) = [];
    
end
function [t_pose_arr, t_rbg_arr, z_mat, position_mat, quat_mat, gt_bb] = load_preprocessed_data(data_dir, datetime_str, conf_thresh, frame_cutoff)
    % load in data specified by the date/time string
    
    fileID = fopen(sprintf('%s/results_%s.txt', data_dir, datetime_str));
    C = textscan(fileID,'%s %f %f %f %f %f');  % name, conyukf.f, min_x, min_y, max_x, max_y
    fclose(fileID);
    num_yolo_outputs = length(C{1}); % NOTE: Can be multple per image!
    
    % variables to hold loaded data    
    t_pose_arr = zeros(1, num_yolo_outputs);
    t_rbg_arr = zeros(1, num_yolo_outputs);
    z_mat = zeros(num_yolo_outputs, 4);    
    position_mat = zeros(num_yolo_outputs, 3);
    quat_mat = zeros(num_yolo_outputs, 4);
    gt_bb = zeros(num_yolo_outputs, 4);
    skipped_outputs = false(1, num_yolo_outputs);
    
    % load the data
    frame_prefix_len = -1;  % # of characters before index (e.g. rbg_03.png --> length(rbg_) = 4)
    for ind = 1:num_yolo_outputs
        conf = C{2}(ind);
        if conf < conf_thresh
            % fprintf("Discarding confidence of %.3f\n", conf);
            skipped_outputs(ind) = true;
            continue;
        end
        frame_name = C{1}{ind};
        if frame_prefix_len < 0
            tokens = strsplit(frame_name, '_');
            frame_prefix_len = length(tokens{1}) + 1;
        end
        frame_ind = str2double(frame_name(frame_prefix_len + 1:end));
        
        if frame_ind > frame_cutoff
            skipped_outputs(ind:end) = true;
            break;
        end
        
        x_min = C{3}(ind); % x is col
        y_min = C{4}(ind); % y is row
        x_max = C{5}(ind);
        y_max = C{6}(ind);
        z_mat(ind, :) = [(y_max + y_min)/2, (x_max + x_min)/2, x_max - x_min, y_max - y_min,]; % [r_center, c_center, width, height]
        
        fileID2 = fopen(sprintf('%s/pose_gtboxes_and_time/pose_gtboxes_and_time_%d.txt', data_dir, frame_ind));
        D = textscan(fileID2,'%f %f %f %f %f %f %f %f %f %f %f %f %f'); % x, y, z, qw, qx, qy, qz, max_coords (c/x_max, r/y_max), min_coords (c/x_min, r/y_min)
        fclose(fileID2);
        
        t_rbg_arr(ind) = D{1}(1);
        t_pose_arr(ind) = D{2}(1);
        position_mat(ind, :) = [D{3}(1), D{4}(1), D{5}(1)];
        quat_mat(ind, :) = [D{6}(1), D{7}(1), D{8}(1), D{9}(1)];
        gt_bb(ind, :) = max_min_coords_to_yolo_bb([D{10}(1), D{11}(1)], [D{12}(1), D{13}(1)]);
    end
    t_pose_arr(skipped_outputs) = [];
    t_rbg_arr(skipped_outputs) = [];
    z_mat(skipped_outputs, :) = [];
    position_mat(skipped_outputs, :) = [];
    quat_mat(skipped_outputs, :) = [];
    gt_bb(skipped_outputs, :) = [];
    
    b_filter_data = true;
    if b_filter_data
        t_diff = t_rbg_arr(2:end) - t_rbg_arr(1:end-1);
        dt_ave = mean(t_diff); fs = 1/dt_ave; fpass = 0.5; % i just made this up!
        fprintf("Average time delta is %.4f seconds\n", dt_ave)
        z_mat_fil = z_mat;
        z_mat_fil(:, 3) = lowpass(z_mat(:, 3), fpass, fs);
        z_mat_fil(:, 4) = lowpass(z_mat(:, 4), fpass, fs);
%         figure(34453434); clf; hold on; 
%         plot(1:length(t_diff), t_diff, 'b.'); 
%         plot(1:length(t_diff), dt_ave*ones(length(t_diff), 1), 'r-'); 
        z_mat = z_mat_fil;
    end
end
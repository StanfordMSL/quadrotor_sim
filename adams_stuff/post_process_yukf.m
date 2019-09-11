function post_process_yukf()   
    close all; clear all; clc; format compact; rng(42);
    global flight k yukf
    fprintf("NOTE: consider using quad offset!!\n")
    
    %%% SCENARIO - Choose to specifiy data & camera position/calibration %%%
    scenario = 3; 
    
    run_dir = sprintf('adams_stuff/preprocessed_data/run%d', scenario);
    hdwr_prms = read_scenario_params(run_dir, scenario);
    data_dir = sprintf('%s/results_%s', run_dir, hdwr_prms.datetime_str);
    
    b_use_perfect_bb = false;
    
    conf_thresh = 0.75; % value below which we decide we did not detect the image
    frame_prefix_len = length('rbg_'); % prefix of first column entries in pose_and_time_###.txt
    b_animate = true;
    b_view_from_camera_perspective = false; % show animation from point of view of camera
    animation_pause = 0.05; % [seconds] amound of extra time to pause between animation frames
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% READ IN PROCESSED DATA %%%%%%%%%
    fileID = fopen(sprintf('%s/results_%s.txt', data_dir, hdwr_prms.datetime_str));
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
    for ind = 1:num_yolo_outputs
        conf = C{2}(ind);
        if conf < conf_thresh
%             fprintf("Discarding confidence of %.3f\n", conf);
            skipped_outputs(ind) = true;
            continue;
        end
        frame_name = C{1}{ind};
        frame_ind = str2double(frame_name(frame_prefix_len + 1:end));
        x_min = C{3}(ind); % x is col
        y_min = C{4}(ind); % y is row
        x_max = C{5}(ind);
        y_max = C{6}(ind);
        z_mat(ind, :) = [(y_max + y_min)/2, (x_max + x_min)/2, x_max - x_min, y_max - y_min]; % [r_center, c_center, width, height]
        
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
    num_img = length(t_pose_arr);
    fprintf("%d images remaining after discarding confidences < %.0f%%\n", num_img, conf_thresh*100);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%% Initialize YUKF %%%
    flight = [];
    flight.x_act = zeros(12, num_img);
    flight.x_act(1:3, :) = position_mat';
    flight.x_act(7:9, :) = quat_mat(:, 2:4)';
    flight.t_act = t_pose_arr;
    %%% YOLO UKF Test Initialization %%%%%%%%%%%%%%%%%%%%
    [sv, yukf] = yolo_ukf_init(flight, t_rbg_arr);
    yukf.hdwr_prms = hdwr_prms;
    camera = init_camera(yukf);
    initial_bb = init_quad_bounding_box('large');
%     initial_bb = init_quad_bounding_box();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % plot various inputs
    compare_yolo_predictions_and_gt(z_mat, gt_bb, position_mat, quat_mat, camera, initial_bb, yukf);
    
    %%% Init plots %%%
    if b_animate
        fig_ani = figure(324234); clf; grid on; hold on
        
        % plot limits
        extra = 3;
        xlim([min([position_mat(:, 1) - extra]),  max([position_mat(:, 1) + extra])]);
        ylim([min([position_mat(:, 2) - extra]),  max([position_mat(:, 2) + extra])]);
        zlim([min([position_mat(:, 3) - extra]),  max([position_mat(:, 3) + extra])]);
        
        % Plot the camera (and expand axis to show it)
        [camera_lims, cmr_h] = plot_camera(camera, fig_ani);
        if b_view_from_camera_perspective
            set(gca, 'CameraPosition', camera.tf_w_cam(1:3, 4)'); 
            view(-90,0); % look from behind the camera
        else
            view(320, 20);
        end
        
        % plot the ground truth trajectory & initialize the estimated trajectory
        traj_h = plot3(position_mat(:, 1), position_mat(:, 2), position_mat(:, 3), 'b-');
        traj_est_h = plot3(NaN, NaN, NaN, 'r-');
        
        %%% plot initial ground truth quad
        % Body Frame Axes
        vect_x = [0.2 0.0 0.0]';
        vect_y = [0.0 0.2 0.0]';
        vect_z = [0.0 0.0 0.1]';

        % Construct Rotation Matrix
        quat = quat_mat(1, :);
        bRw = quat2rotm(quat);

        % Determine World Frame Pose of Craft Axes
        pos = position_mat(1, :)';

        x_arrow = [pos pos + (bRw*vect_x)];
        y_arrow = [pos pos + (bRw*vect_y)];
        z_arrow = [pos pos + (bRw*vect_z)];

        x = [x_arrow(1,:); y_arrow(1,:); z_arrow(1,:)]';
        y = [x_arrow(2,:); y_arrow(2,:); z_arrow(2,:)]';
        z = [x_arrow(3,:); y_arrow(3,:); z_arrow(3,:)]';

        % Plot It!
        h_persp = plot3(x, y, z, 'linewidth', 3);

        % Set the Correct Colors
        h_persp(1).Color = [1 0 0];
        h_persp(2).Color = [0 1 0];
        h_persp(3).Color = [0 0 1];
        %%%%%%%%%%%
        
        %%% plot initial estimated quad
        % Body Frame Axes
        vect_x_est = [0.2 0.0 0.0]';
        vect_y_est = [0.0 0.2 0.0]';
        vect_z_est = [0.0 0.0 0.1]';

        % Construct Rotation Matrix
        quat_est = complete_unit_quat(sv.mu_hist(7:9, 1));
        bRw_est = quat2rotm(quat_est');

        % Determine World Frame Pose of Craft Axes
        pos_est = sv.mu_hist(1:3, 1);

        x_arrow_est = [pos_est pos_est + (bRw_est*vect_x_est)];
        y_arrow_est = [pos_est pos_est + (bRw_est*vect_y_est)];
        z_arrow_est = [pos_est pos_est + (bRw_est*vect_z_est)];

        x_est = [x_arrow_est(1,:); y_arrow_est(1,:); z_arrow_est(1,:)]';
        y_est = [x_arrow_est(2,:); y_arrow_est(2,:); z_arrow_est(2,:)]';
        z_est = [x_arrow_est(3,:); y_arrow_est(3,:); z_arrow_est(3,:)]';

        % Plot It!
        h_persp_est = plot3(x_est, y_est, z_est, 'linewidth', 3);

        % Set the Correct Colors
        h_persp_est(1).Color = [0.8 0 0];
        h_persp_est(2).Color = [0 0.8 0];
        h_persp_est(3).Color = [0 0 0.8];
        %%%%%%%%%%%%%
        
        % set the labels & legend  
        xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis');
        legend([cmr_h; traj_h; traj_est_h; h_persp; h_persp_est], {'camera', 'traj._{gt}', 'traj._{est}', 'X_{gt}','Y_{gt}','Z_{gt}', 'X_{est}','Y_{est}','Z_{est}'});
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %%% YUKF %%%%%%%%%
%     flight.x_act = zeros(12, num_img);
%     flight.x_act(1:3, :) = position_mat';
%     flight.x_act(7:9, :) = quat_mat(:, 2:4)';
    for k = 1:num_img
        % YOLO UKF %%%%%%
        if(k > 1)
            t_now = t_rbg_arr(k);
            yukf.dt = t_now - t_rbg_arr(k - 1);
            
            if b_use_perfect_bb
                [sv, yukf] = yolo_ukf(yukf, sv, flight, k, t_now, initial_bb, camera, [], []);
            else
                yolo_output = [z_mat(k, :)'; 0]; 
%                 yolo_output = predict_quad_bounding_box(flight.x_act(:, k), camera, initial_bb, yukf); %"perfect" prediction
                % [gt_bb(k, :)'; 0];
                yukf = yukf_step(yukf, [], yolo_output, [], camera, initial_bb);
                
                % Save values for plotting %%%%%%%%%%%%%%%%%%
                sv.mu_hist(:, k) = yukf.mu;
                sv.mu_act(:, k) = flight.x_act(:, k);

                sv.sig_hist(:, :, k) = yukf.sigma;
                sv.sig_trace_hist(k) = trace(yukf.sigma);
                sv.time_hist(k) = t_now;
                sv.hist_mask(k) = true;

                qm = complete_unit_quat(yukf.mu(7:9)); 
                [yaw, pitch, roll] = quat2angle(qm(:)');
                sv.ypr_hist(:, k) = [yaw; pitch; roll]*180/pi;

                qa = complete_unit_quat(flight.x_act(7:9, k)); 
                [yaw, pitch, roll] = quat2angle(qa(:)');
                sv.ypr_act_hist(:, k) = [yaw; pitch; roll]*180/pi;

                sv.ang_err(k) = quat_dist(qa, qm);
                sv.ang(k) = 2*acosd(qm(1));
                sv.ang_act(k) = 2*acosd(qa(1));
            end
        end
        %%%%%%%%%%%%%%%%%%%
        if b_animate
            disp('')
            % update ground truth quad pose
            quat = quat_mat(k, :);
            bRw = quat2rotm(quat);
            pos = position_mat(k, :)';

            x_arrow = [pos pos + (bRw*vect_x)];
            y_arrow = [pos pos + (bRw*vect_y)];
            z_arrow = [pos pos + (bRw*vect_z)];
            h_persp = reassign(h_persp, x_arrow, y_arrow, z_arrow);
            
            % update estimate trajectory plot
            set(traj_est_h, 'xdata', sv.mu_hist(1, sv.hist_mask), ...
                            'ydata', sv.mu_hist(2, sv.hist_mask), ...
                            'zdata', sv.mu_hist(3, sv.hist_mask) );
            
            % update estimated quad pose
            quat_est = complete_unit_quat(sv.mu_hist(7:9, k));
            bRw_est = quat2rotm(quat_est');
            pos_est = sv.mu_hist(1:3, k);

            x_arrow_est = [pos_est pos_est + (bRw_est*vect_x_est)];
            y_arrow_est = [pos_est pos_est + (bRw_est*vect_y_est)];
            z_arrow_est = [pos_est pos_est + (bRw_est*vect_z_est)];
            h_persp_est = reassign(h_persp_est, x_arrow_est, y_arrow_est, z_arrow_est);
            pause(animation_pause)
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot history of filter vs ground truth
    disp('')
    plot_ukf_hist(sv, flight);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp('')
end
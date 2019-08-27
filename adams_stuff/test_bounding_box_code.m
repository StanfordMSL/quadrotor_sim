function test_bounding_box_code(varargin)

    %%% PARAMS %%%
    yukf.prms.b_use_control = true;  % whether to use the control in our estimate
    %%%% OPTIONS FOR SENSOR %%%%%%%%%%%%%%%%%%%%%%%%
    % Option 1 %%%%%%%   z = [row, col, width, height, angle]
    yukf.prms.b_angled_bounding_box = false; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 2 %%%%%%%   z = [state]
    yukf.prms.b_measure_everything = false; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 3 (DEFAULT) %%%%%%%   z = [[row, col, width, height, <extra1>, <extra2>, ...]
    yukf.prms.b_measure_aspect_ratio = false; % when not angled, this will include a 5th value (ratio of height to width of bounding box)
    % ___extra A
    yukf.prms.b_measure_yaw = false;
    % ___extra A
    yukf.prms.b_measure_pitch = false;
    % ___extra A
    yukf.prms.b_measure_roll = false;
    % ___extra B
    yukf.prms.b_measure_x = false;
    % ___extra C
    yukf.prms.b_measure_quat = false;
    % ___extra D
    yukf.prms.b_measure_pos = false;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    initial_bb = init_quad_bounding_box();
    camera = init_camera();
    if ~isempty(varargin)
        x_curr = varargin{1};
    else
        clc; format compact; rng(42);

        % angle defining R_w_quad
        r = 0;
        p = 7.5 * pi/180;
        y = 0;
        R_test = eul2rotm([r, p, y]); % == R_w_quad
        q_test = angle2quat(r, p, y)';

        x_curr = [0; 0; 1; 0; 0; 0; q_test(2:4); 0; 0; 0];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % unpack state
    pos_w = x_curr(1:3, 1);
    vel = x_curr(4:6, 1);
    quat = complete_unit_quat(x_curr(7:9, 1));
    wx = x_curr(10, 1);
    wy = x_curr(11, 1);
    wz = x_curr(12, 1);
    
    R_w_quad = quat2rotm(quat(:)'); % I confirmed this
    tf_w_quad = [R_w_quad, pos_w(:); [zeros(1, 3), 1]];
    
    tf_cam_quad = camera.tf_cam_w * tf_w_quad;
    
    num_verts = size(initial_bb, 1);
    bb_cam = (tf_cam_quad * [initial_bb, ones(num_verts, 1)]')';
    bb_cam = bb_cam(:, 1:3);
    
    %%% plot 3D box
    plot_3D_bb_world(initial_bb, tf_w_quad, pos_w)
    set(gca, 'CameraPosition', camera.tf_w_cam(1:3, 4)); % set plot camera to be where actual camera is
    
    [output, bb_rc_list] = predict_quad_bounding_box(x_curr, camera, initial_bb, yukf);
    [yaw, pitch, roll] = quat2angle(quat(:)');
    
    disp('')
    % plot 2D projection of 3D box AND 2D BB
    if yukf.prms.b_angled_bounding_box
        plot_bounding_angled_box(output(1:2), output(3), output(4), output(5), bb_rc_list, pos_w, quat, camera)
    else
        plot_bounding_aligned_box(output(1:2), [output(4), output(3)], bb_rc_list, pos_w, quat, camera);
    end
    
    disp('')
end
    
%     quad_aligned_bb = [l/2, w/2, h/2;... %    1 front, left, up (from quad's perspective)
%                        l/2, -w/2, h/2;... %   2 front, right, up
%                        -l/2, -w/2, h/2;... %  3 back, right, up
%                        -l/2, w/2, h/2; ... %  4 back, left, up
%                        l/2, w/2, -h/2;... %   5 front, left, down
%                        l/2, -w/2, -h/2;... %  6 front, right, down
%                        -l/2, -w/2, -h/2;... % 7 back, right, down
%                        -l/2, w/2, -h/2 ]; %   8 back, left, down
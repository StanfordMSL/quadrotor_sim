function test_bounding_box_code(varargin)
    global yukf

    num_dims = 12; % length of state
    flight.x_act = zeros(num_dims, 1);  % this is a placeholder that needs to happen before yolo_yukf_init()
    yukf = yolo_ukf_init(num_dims, NaN); % this sets most of the filter parameters, the rest are loaded from a file

    scenario = 5; 
    run_dir = sprintf('adams_stuff/preprocessed_data/run%d', scenario);
    yukf.hdwr_prms = read_scenario_params(run_dir, scenario);
    data_dir = sprintf('%s/results_%s', run_dir, yukf.hdwr_prms.datetime_str);
    
    initial_bb = init_quad_bounding_box(yukf.hdwr_prms.bb_l, yukf.hdwr_prms.bb_w, yukf.hdwr_prms.bb_h, yukf.hdwr_prms.bb_mult);
    camera = init_camera(yukf);

    if ~isempty(varargin)
        x_curr = varargin{1};
    else
        clc; format compact; rng(42);

        % angle defining R_w_quad
        r = 10 * pi/180;
        p = 1*7.5 * pi/180;
        y = 3 * pi/180;
        q_test = angle2quat(r, p, y, 'XYZ')';

        x_curr = [0; 0; 1; 0; 0; 0; q_test(2:4); 0; 0; 0];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % unpack state
    pos_w = x_curr(1:3, 1);
    vel = x_curr(4:6, 1);
    quat = x_curr(7:10, 1); % [y,p,r] = quat2angle(quat(:)')
    wx = x_curr(11, 1);
    wy = x_curr(12, 1);
    wz = x_curr(13, 1);
    
    R_w_quad = quat2rotm(quat(:)'); % I confirmed this
    tf_w_quad = [R_w_quad, pos_w(:); [zeros(1, 3), 1]];
    
    tf_cam_quad = camera.tf_cam_w * tf_w_quad;
    
    num_verts = size(initial_bb, 1);
    bb_cam = (tf_cam_quad * [initial_bb, ones(num_verts, 1)]')';
    bb_cam = bb_cam(:, 1:3);
    
    %%% plot 3D box
    plot_3D_bb_world(initial_bb, tf_w_quad, pos_w, camera.tf_w_cam(1:3, 4))
    
    [output, bb_rc_list] = predict_quad_bounding_box(x_curr, camera, initial_bb, yukf);
    [roll, pitch, yaw] = quat2angle(quat(:)', 'XYZ');
    
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
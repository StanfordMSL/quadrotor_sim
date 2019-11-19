function camera = init_camera(varargin)
    % define camera
    if ~isempty(varargin)
        yukf = varargin{1};
    end
    if exist('yukf', 'var') ~= 1 || ~isfield(yukf, 'hdwr_prms') || ~isfield(yukf.hdwr_prms, 'camera_scenario')
        % default (and sim-only) scenario
        warning("using default camera settings - dont use with real data!");
        camera_position_w = [-8, 0, 1]; % world frame
        ang_x = 90; ang_y = 0; ang_z = 90;
        Rx = [ 1.  , 0.          ,     0.       ; 0.  , cosd(ang_x) ,-sind(ang_x)  ; 0.  , sind(ang_x) , cosd(ang_x)  ];
        Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  0.  ,         1.       , 0            ; -sind(ang_y) , 0.   , cosd(ang_y)  ];
        Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ; sind(ang_z) , cosd(ang_z) , 0.  ;  0.              , 0.        1.   ];
        R_cam_w = Rx * Ry * Rz;
        R_w_cam = R_cam_w';
        camera_quat = rotm2quat(R_w_cam);
        camera_pose = [camera_position_w, camera_quat]; % [x,y,z, quat] in world frame 
        camera.quad_offset_w = [0, 0, 0];  

        % store values in struct
        camera.pose = camera_pose;
        camera.tf_w_cam = [R_w_cam, camera_position_w(:); [zeros(1, 3), 1]];
        camera.tf_cam_w = [R_w_cam', -R_w_cam'*camera_position_w(:); [zeros(1, 3), 1]];
    else
        %%% Use hardware params loaded from file
        quat = yukf.hdwr_prms.camera_quat_w(:)';
        quat = normalize_quat(quat);
        tf_w_copt = zeros(4, 4);
        tf_w_copt(1:3, 4) = yukf.hdwr_prms.camera_pos_w(:);
        tf_w_copt(1:3, 1:3) = quat2rotm(quat);

        % this is manual calibration for turning copt frame to camera frame
        tf_cam_copt = tf_from_rpy_and_trans(yukf.hdwr_prms.camera_rot_x, ...
                                            yukf.hdwr_prms.camera_rot_y, ...
                                            yukf.hdwr_prms.camera_rot_z, ...
                                            yukf.hdwr_prms.camera_shift_w);

        % store values in struct
        camera.tf_w_cam = tf_w_copt * inv_tf(tf_cam_copt);
        camera.tf_cam_w = inv_tf(camera.tf_w_cam);
        camera.pose = [camera.tf_w_cam(1:3, 4)', rotm2quat(camera.tf_w_cam(1:3, 1:3))];
    end

    %%% Set Camera intrinsics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    K_image_to_pix =  [617.2744140625, 0.0,              324.1011047363281, 0.;
                       0.0,            617.335693359375, 241.5790557861328, 0.;
                       0.0,            0.0,              1.0              , 0.];

    camera.K = K_image_to_pix(1:3, 1:3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    disp('');
    % for plotting only
    len = 0.5;
    camera.draw_len = len;
    cam_width = len / K_image_to_pix(1,1) * K_image_to_pix(1, 3)*2;
    cam_height = len / K_image_to_pix(2,2) * K_image_to_pix(2, 3)*2;
    camera.viz_wf = [0, 0, 0; cam_width/2, cam_height/2, len; NaN, NaN, NaN; ...
                     0, 0, 0; -cam_width/2, cam_height/2, len; NaN, NaN, NaN;  ...
                     0, 0, 0; cam_width/2, -cam_height/2, len; NaN, NaN, NaN;  ...
                     0, 0, 0; -cam_width/2, -cam_height/2, len; NaN, NaN, NaN;  ...
                     -cam_width/2, -cam_height/2, len; -cam_width/2, cam_height/2, len; NaN, NaN, NaN;  ...
                     cam_width/2, -cam_height/2, len; cam_width/2, cam_height/2, len; NaN, NaN, NaN;  ...
                     -cam_width/2, -cam_height/2, len; cam_width/2, -cam_height/2, len; NaN, NaN, NaN;  ...
                     -cam_width/2, cam_height/2, len; cam_width/2, cam_height/2, len];
                 
   camera.tf_cam_quadego = eye(4);
   camera.tf_quadego_cam = eye(4);
end

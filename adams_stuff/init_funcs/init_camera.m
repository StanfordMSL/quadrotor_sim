function camera = init_camera()
    % define camera
    
    % set(gca, 'CameraPosition', [-8,0,0]); view(90,0); % <-- to look in direction camera is looking
    
    % where is the camera
    camera_position_w = [-8, 0, 0]; % world frame
    ang_x = 90;
    ang_y = 0;
    ang_z = 90;
    Rx = [ 1.  , 0.          ,     0.       ;
           0.  , cosd(ang_x) ,-sind(ang_x)  ;
           0.  , sind(ang_x) , cosd(ang_x)  ];
               
    Ry = [ cosd(ang_y) , 0.    , sind(ang_y)  ;
          0.  ,         1.       , 0            ;
          -sind(ang_y) , 0.   , cosd(ang_y)  ];
               
    Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ;
           sind(ang_z) , cosd(ang_z) , 0.  ;
           0.              , 0.        1.   ];
    R_cam_w = Rx * Ry * Rz;
    R_w_cam = R_cam_w';
    camera_quat = rotm2quat(R_w_cam);
    camera_pose = [camera_position_w, camera_quat]; % [x,y,z, quat] in world frame    
    
    % camera intrinsics
    K_image_to_pix =  [617.2744140625, 0.0,              324.1011047363281, 0.;
                       0.0,            617.335693359375, 241.5790557861328, 0.;
                       0.0,            0.0,              1.0              , 0.];
    % store values in struct
    camera.pose = camera_pose;
    camera.K_3x3 = K_image_to_pix(1:3, 1:3);
    camera.K_3x4 = K_image_to_pix;
%     R_cam_w = quat2rotm(camera_pose(4:7)');
    camera.tf_w_cam = [R_w_cam, camera_position_w(:); [zeros(1, 3), 1]];
    camera.tf_cam_w = [R_w_cam', -R_w_cam'*camera_position_w(:); [zeros(1, 3), 1]];
    
    % for plotting only
    len = 0.75;
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
end
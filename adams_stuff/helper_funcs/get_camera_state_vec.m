function cam_state = get_camera_state_vec(camera)
% camera_pose = [camera_position_w, camera_quat]; % [x,y,z, quat] in world frame    
    cam_state(1:3, 1) = camera.pose(1:3);  
    cam_state(4:6, 1) = [0; 0; 0];
    cam_state(7:9, 1) = camera.pose(5:7);
    cam_state(10:12, 1) = [0; 0; 0];
end
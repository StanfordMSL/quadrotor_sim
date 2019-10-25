function b_in_view = is_point_in_view(wp_x, wp_y, wp_z, camera, yukf)
    pix_buffer = 15;
    rcs(1, :) = xyz_to_rc(camera.tf_cam_w * [wp_x - yukf.hdwr_prms.bb_l/2, wp_y - yukf.hdwr_prms.bb_w/2, wp_z - yukf.hdwr_prms.bb_h/2, 1]', camera);
    rcs(2, :) = xyz_to_rc(camera.tf_cam_w * [wp_x - yukf.hdwr_prms.bb_l/2, wp_y + yukf.hdwr_prms.bb_w/2, wp_z + yukf.hdwr_prms.bb_h/2, 1]', camera);
    b_in_view = all(rcs(:) > pix_buffer) && all(rcs(:, 1) < (480 - pix_buffer)) && all(rcs(:, 2) < (640 - pix_buffer));
end
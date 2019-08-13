function plot_camera(camera, fig_h)
    figure(fig_h);
    xyzs = (camera.tf_w_cam * [camera.viz_wf, ones(size(camera.viz_wf, 1), 1)]')';
    plot3(xyzs(:, 1), xyzs(:, 2), xyzs(:,3), 'b-')
end
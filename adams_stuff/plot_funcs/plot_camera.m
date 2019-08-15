function [lims, plt_h] = plot_camera(camera, fig_h)
    figure(fig_h);
    xyzs = (camera.tf_w_cam * [camera.viz_wf, ones(size(camera.viz_wf, 1), 1)]')';
    lims = [min(xyzs(:,1:3)); max(xyzs(:,1:3))];
    plt_h = plot3(xyzs(:, 1), xyzs(:, 2), xyzs(:,3), 'm-');
end
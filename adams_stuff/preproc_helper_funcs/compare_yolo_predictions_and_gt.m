function compare_yolo_predictions_and_gt(yolo_bb, gt_bb, position_mat, quat_mat, camera, initial_bb, yukf)
    %%% plot yolo vs prediction vs gt %%%
    num_img = size(yolo_bb, 2);
    bb_yolo = yolo_bb';
    bb_pred = zeros(num_img, 4);
    bb_gt = gt_bb;
    
    state_tmp = zeros(12, num_img);
    state_tmp(1:3, :) = position_mat';
    state_tmp(7:9, :) = quat_mat(:, 2:4)';
    
    for k = 1:num_img
        bb_pred_tmp = predict_quad_bounding_box(state_tmp(:, k), camera, initial_bb, yukf)'; 
        bb_pred(k, :) = bb_pred_tmp(1:4); % might have a 0 yaw tacked on at end depending on settings
    end
    figure(4545); clf; xx = 0:(num_img-1);
    ylabs = {'row', 'col', 'width', 'height'};
    for ii = 1:4
        subplot(2, 2, ii); hold on; ylabel(ylabs{ii}); xlabel('Image # (0 based)');
        a = plot(xx, bb_yolo(:, ii), 'r-'); % yolo actual output
        b = plot(xx, bb_pred(:, ii), 'g-'); % my simulated output
        c = plot(xx, bb_gt(:, ii), 'b-'); % ground truth (from optitrack)
        title(ylabs{ii})
        legend([a,b,c], {"yolo", "predicted", "truth"});
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bb = max_min_coords_to_yolo_bb(max_coords, min_coords)
    % using the way min/max coords are defined in Q's code... i.e. [x,y] = [c, r]
    center_r = (max_coords(2) + min_coords(2)) / 2; % average y value
    center_c = (max_coords(1) + min_coords(1)) / 2; % average x value
    width = max_coords(1) - min_coords(1);
    height = max_coords(2) -min_coords(2);
    bb = [center_r, center_c, width, height]; % max coords, min_coords
end
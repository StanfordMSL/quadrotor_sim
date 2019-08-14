function bb = init_quad_bounding_box()
    % define quad
    l = 0.44*1.2;
    w = 0.44*1.2;
    h = 0.2*1.2;
    quad_aligned_bb = [l/2, w/2, h/2;... % front, left, up (from quad's perspective)
                       l/2, -w/2, h/2;... % front, right, up
                       -l/2, -w/2, h/2;... % back, right, up
                       -l/2, w/2, h/2; ... % back, left, up
                       l/2, w/2, -h/2;... % front, left, down
                       l/2, -w/2, -h/2;... % front, right, down
                       -l/2, -w/2, -h/2;... % back, right, down
                       -l/2, w/2, -h/2 ]; % back, left, down
   bb = quad_aligned_bb;
end
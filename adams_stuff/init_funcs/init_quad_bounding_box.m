function bb = init_quad_bounding_box()
    % define quad
    l = 0.44*1.2;
    w = 0.44*1.2;
    h = 0.2*1.2;
            
    quad_aligned_bb = [l/2, w/2, h/2;... %    1 front, left, up (from quad's perspective)
                       l/2, -w/2, h/2;... %   2 front, right, up
                       -l/2, -w/2, h/2;... %  3 back, right, up
                       -l/2, w/2, h/2; ... %  4 back, left, up
                       l/2, w/2, -h/2;... %   5 front, left, down
                       l/2, -w/2, -h/2;... %  6 front, right, down
                       -l/2, -w/2, -h/2;... % 7 back, right, down
                       -l/2, w/2, -h/2 ]; %   8 back, left, down
                   
   bb = quad_aligned_bb;
end
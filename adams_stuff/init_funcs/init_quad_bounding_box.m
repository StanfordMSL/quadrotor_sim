function bb = init_quad_bounding_box(l, w, h, mult)

%     % define quad (give a little extra space to allow for boundary width, i.e. x1.2)
%     if ~isempty(varargin) && (strcmpi(varargin{1}, 'old') || strcmpi(varargin{1}, 'large'))
%         % Q's original values: (used for camera scenarios 1-3)
%         fprintf("CAREFUL: Using Large Bounding Box!\n");
%         l = 0.44*1.;
%         w = 0.44*1.;
%         h = 0.2*1.;
%     else
%         l = 0.27*1.2;
%         w = 0.27*1.2;
%         h = 0.13*1.2;
%     end

    l = l * mult;
    h = h * mult;
    w = w * mult;
            
    quad_aligned_bb = [l/2,  w/2,  h/2;... %  1 front, left, up (from quad's perspective)
                       l/2, -w/2,  h/2;... %  2 front, right, up
                      -l/2, -w/2,  h/2;... %  3 back, right, up
                      -l/2,  w/2,  h/2;... %  4 back, left, up
                       l/2,  w/2, -h/2;... %  5 front, left, down
                       l/2, -w/2, -h/2;... %  6 front, right, down
                      -l/2, -w/2, -h/2;... %  7 back, right, down
                      -l/2,  w/2, -h/2];   %  8 back, left, down
   
   bb = quad_aligned_bb;
end
function bb_box_xy = bb_5val_to_xys(bb_5_el)
    r = bb_5_el(1);
    c = bb_5_el(2);
    w = bb_5_el(3);
    h = bb_5_el(4);
    ang = bb_5_el(5); % in rad
    rot_mat = [ cos(ang), -sin(ang);
                sin(ang),  cos(ang) ];
    bb_box_xy = [c, r] + ...
       ( rot_mat' * ...  % NOTE: transpose here is because we are flipping the y axis
        [-w/2,   -h/2;
          w/2, -h/2;
          w/2,  h/2;
         -w/2,  h/2]')';
end
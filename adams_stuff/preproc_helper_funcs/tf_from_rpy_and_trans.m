function tf = tf_from_rpy_and_trans(ang_x, ang_y, ang_z, trans)
    Rx = [ 1.  , 0.          ,     0.        ; 
           0.  , cosd(ang_x) ,-sind(ang_x)   ; 
           0.  , sind(ang_x) , cosd(ang_x)  ];
    Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  
          0.  ,         1.       , 0         ; 
         -sind(ang_y) , 0.   , cosd(ang_y)  ];
    Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.    ; 
           sind(ang_z) , cosd(ang_z) , 0.    ;  
           0.          ,     0.        1.   ];
    R_cam_copt = Rx * Rz;
    tf = trans(:);
    tf = zeros(4, 4);
    tf(1:3, 1:3) = R_cam_copt;
    tf(1:3, 4) = R_cam_copt' * -t_copt_cam;
end
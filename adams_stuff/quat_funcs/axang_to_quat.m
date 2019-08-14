function quat = axang_to_quat(axang)
    % input is a 3 element pertubation vector in axis-angle format
    
    ang = norm(axang);
    if(abs(ang) > 0.001)
        vec_perturb = axang/ang;
        quat = [cos(ang/2); vec_perturb(:)*sin(ang/2)];
    else
        quat = [1;0;0;0];
    end
end
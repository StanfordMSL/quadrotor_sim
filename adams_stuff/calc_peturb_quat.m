function quat_perturb = calc_peturb_quat(axang_perturb)
    % input is a 3 element pertubation vector in axis-angle format
    
    ang_perturb = norm(axang_perturb);
    if(abs(ang_perturb) > 0.001)
        vec_perturb = axang_perturb/ang_perturb;
        quat_perturb = [cos(ang_perturb/2); vec_perturb(:)*sin(ang_perturb/2)];
    else
        quat_perturb = [1;0;0;0];
    end
end
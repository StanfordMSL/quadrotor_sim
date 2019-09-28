function predictParticles = stateTransition(pf,prevParticles,model,u,dt,process_noise)
%STATETRANSITION Summary of this function goes here

%   Detailed explanation goes here
    predictParticles = zeros(pf.NumParticles,pf.NumStateVariables);
    for part_ind = 1:pf.NumParticles
        predictParticles(part_ind,:) = propagate_state(prevParticles(part_ind,:), model, u, dt);
        predictParticles_ax_ang = predictParticles(part_ind,:)';
        quat = complete_unit_quat(predictParticles_ax_ang(7:9));
        predictParticles_ax_ang(7:9) = quat_to_axang(quat) ;
        predictParticles_ax_ang = normrnd(predictParticles_ax_ang,diag(process_noise));
        predictParticles(part_ind,:) = predictParticles_ax_ang;
        quat = axang_to_quat(predictParticles_ax_ang(7:9));
        predictParticles(part_ind,7:9) = quat(2:4);
        
    end
    % Keep the best particle
    predictParticles(1,:) = propagate_state(pf.State, model, u, dt);
    predictParticles(2,:) = pf.State;

end


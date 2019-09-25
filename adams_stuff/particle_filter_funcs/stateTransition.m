function predictParticles = stateTransition(pf,prevParticles,model,u,dt,process_noise)
%STATETRANSITION Summary of this function goes here
%   Detailed explanation goes here
    predictParticles = zeros(pf.NumParticles,pf.NumStateVariables);
    for part_ind = 1:pf.NumParticles
        predictParticles(part_ind,:) = propagate_state(prevParticles(part_ind,:), model, u, dt);
        predictParticles(part_ind,:) = normrnd(predictParticles(part_ind,:)',diag(process_noise));
    end

end


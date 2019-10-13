function predictParticles = stateTransition(pf,prevParticles,model,u,dt,process_noise)
%STATETRANSITION Summary of this function goes here

%   Detailed explanation goes here
    predictParticles_quat = zeros(1,13);

    axang_3_dim = prevParticles(:,7:9);
    ang_val = sqrt(sum(axang_3_dim.^2,2));
    prevParticles_quat = [prevParticles(:,1:6) axang2quat([axang_3_dim./ang_val ang_val]) prevParticles(:,10:12)];
    num_part = pf.NumParticles;
    random_acc = rand(4,num_part)*2;
    model  = model_init('simple vII',0,0,0);
    parfor part_ind = 1:num_part
        u = random_acc(:,part_ind);
        predictParticles_quat = propagate_state(prevParticles_quat(part_ind,:), model, u, dt);
        predictParticles_ax_ang = zeros(12,1);
        predictParticles_ax_ang(1:6) = predictParticles_quat(1:6)';
        predictParticles_ax_ang(10:12) = predictParticles_quat(11:13)';
        quat = predictParticles_quat(7:10)';
        predictParticles_ax_ang(7:9) = quat_to_axang(quat) ;
        predictParticles_ax_ang = normrnd(predictParticles_ax_ang,diag(process_noise));
        predictParticles(part_ind,:) = predictParticles_ax_ang;
%         predictParticles(part_ind,7:10) = axang_to_quat(predictParticles_ax_ang(7:9));
%         
    end
    % Keep the best particle
%     predictParticles(1,:) = propagate_state(pf.State, model, u, dt);
%     predictParticles(2,:) = pf.State;

end


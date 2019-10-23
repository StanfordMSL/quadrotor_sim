function predictParticles = stateTransition(pf,prevParticles,model,u,dt,process_noise,reset_acc)
%STATETRANSITION Summary of this function goes here

%   Detailed explanation goes here
    predictParticles_quat = zeros(1,13);
    if reset_acc
        prevParticles = repmat(pf.State,pf.NumParticles,1);
    end
    axang_3_dim = prevParticles(:,7:9);
    ang_val = sqrt(sum(axang_3_dim.^2,2));
    prevParticles_quat = [prevParticles(:,1:6) axang2quat([axang_3_dim./ang_val ang_val]) prevParticles(:,10:12)];
    num_part = pf.NumParticles;
    model  = model_init('simple vII',0,0,0);
    %random_acc = rand(4,num_part)*2+model.hover_u;
    %a_z = (rand(1,num_part)*2*model.m_act*model.g-model.m_act*model.g)+model.m_act*model.g;
    nb_each = num_part^(1/3);
    a_z = model.m_act*model.g + (-1*model.m_act*model.g:2*model.m_act*model.g/nb_each:1*model.m_act*model.g);
    torque_val = pi/2;
    omega_dot_r = -torque_val:torque_val*2/nb_each:torque_val;
    omega_dot_p = -torque_val:torque_val*2/nb_each:torque_val;
    combinations = allcomb(a_z,omega_dot_r,omega_dot_p)';
    a_z = combinations(1,:);
%    omega_dot = [zeros(3,ceil(nb_each)^3)];
    omega_dot = [combinations(2:3,:);zeros(1,ceil(nb_each)^3)];

    % omega_dot = (rand(3,num_part)*pi/2-pi/4);
    omega = prevParticles(10:12);
    parfor part_ind = 1:num_part
        u = quad_inputs_from_acc(a_z(part_ind),omega_dot(:,part_ind),omega,model);
        %u = random_acc(:,part_ind);
        %predictParticles_quat = propagate_state(prevParticles_quat(part_ind,:), model, u, dt);
        predictParticles_quat = propagate_state(prevParticles_quat(part_ind,:), model, u, dt);
        predictParticles_ax_ang = zeros(12,1);
        predictParticles_ax_ang(1:6) = predictParticles_quat(1:6)';
        predictParticles_ax_ang(10:12) = predictParticles_quat(11:13)';
        quat = predictParticles_quat(7:10)';
        predictParticles_ax_ang(7:9) = quat_to_axang(quat) ;
        predictParticles_ax_ang = normrnd(predictParticles_ax_ang,diag(process_noise)*3);
        predictParticles(part_ind,:) = predictParticles_ax_ang;
%         predictParticles(part_ind,7:10) = axang_to_quat(predictParticles_ax_ang(7:9));
%         
    end
    % Keep the best particle
%     predictParticles(1,:) = propagate_state(pf.State, model, u, dt);
%     predictParticles(2,:) = pf.State;

end


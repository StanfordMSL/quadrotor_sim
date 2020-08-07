function [x_bar,u_bar,R_gamma] = ileqr_oa_fp(x_bar,u_bar,x_now,l,L,model,Q_t,Q_f,R,coeff_obs)
    % Initialize some terms
    N = size(x_bar,2);
    x_fp = zeros(13,N);
    x_fp(:,1) = x_now;
    u_fp = u_bar; 
    J =  0;
    
for k = 1:N-1
    % Determine Control Command
    del_x = x_fp(:,k)-x_bar(:,k);
    del_u = model.alpha.*(l(:,:,k) + L(:,:,k)*del_x);
    u_fp(:,k) = u_fp(:,k) + del_u;

    % Predict Dynamics of Next Step
    FT_ext = zeros(6,1);
    m_cmd = wrench2m_controller(u_fp(:,k),model);

    x_fp(:,k+1) = quadcopter(x_fp(:,k),m_cmd,model,FT_ext,'ctl');

    % Update Cost
    dist1 = norm(x_fp(1:3,k)-[0 0 1.05]');
    dist2 = norm(x_fp(1:3,k)-[0 0 1.0]');

    J_obs1 = coeff_obs(1,1)*(coeff_obs(1,2)*dist1)^(-2);
    J_obs2 = coeff_obs(2,1)*(coeff_obs(2,2)*dist2)^(-2);
    J = J + 0.5*(del_x'*Q_t*del_x + u_fp(:,k)'*R*u_fp(:,k)) + J_obs1 + J_obs2;
%     dist1 = norm(x_fp(1:3,k)-[0 0 1.00]');
%     J_obs1 = coeff_obs(1,1)*(coeff_obs(1,2)*dist1)^(-3);
%     J = J + 0.5*(del_x'*Q_t*del_x + u_fp(:,k)'*R*u_fp(:,k)) + J_obs1;
end

% Add terminal cost   
del_x = x_fp(:,N)-x_bar(:,N);

dist1 = norm(x_fp(1:3,N)-[0 0 1.05]');
dist2 = norm(x_fp(1:3,N)-[0 0 1]');

J_obs1 = coeff_obs(1,1)*(coeff_obs(1,2)*dist1)^(-2);
J_obs2 = coeff_obs(2,1)*(coeff_obs(2,2)*dist2)^(-2);

J = J + 0.5* del_x'*Q_f*del_x + J_obs1 + J_obs2;
% dist1 = norm(x_fp(1:3,N)-[0 0 1.00]');
% J_obs1 = coeff_obs(1,1)*(coeff_obs(1,2)*dist1)^(-3);
% J = J + 0.5*(del_x'*Q_t*del_x + u_fp(:,k)'*R*u_fp(:,k)) + J_obs1;
    
% Convert to LEQR form
R_gamma = (1/model.gamma).* log(exp(model.gamma .* J));
% disp(['[ilq_fp]: Current Cost: ',num2str(cost_curr)]);

% If cost goes down, we know it's feasible. Update x_bar.
x_bar = x_fp;
u_bar = u_fp;

% u_test = sum(isnan(u_bar),1:2);   
% if (u_test > 0)
%     disp('[ilqr_fp]: NaN detected in u.')
% end

end
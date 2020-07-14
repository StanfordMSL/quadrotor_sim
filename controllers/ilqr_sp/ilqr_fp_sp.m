function [x_bar,u_bar,J] = ilqr_fp_sp(x_curr,x_wp,x_bar,u_bar,l,L,wts,model)

% Unpack Stuff
Q_t = wts.Q_pstn;
Q_f = wts.Q_unif;
R   = wts.R_stnd;

dt_fmu = model.dt_fmu;
dt_ctl = model.dt_ctl;
N_ctl = model.N_ctl;

% Initialize some terms
tf   = N_ctl .* model.dt_ctl;
N_fmu = round(tf.*model.hz_fmu)+1;
 
x_fmu = zeros(13,N_fmu);
x_fmu(:,1) = x_curr;
J =  0;
  
k_ctl = 1;
for k_fmu = 1:N_fmu-1
    t_now = (k_fmu-1)*dt_fmu;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Feedback Law Updater
    if mod(t_now,dt_ctl) == 0        
        [u_cmd,m_cmd] = fbc(x_fmu(:,k_fmu),x_bar(:,k_ctl),u_bar(:,k_ctl),l(:,:,k_ctl),L(:,:,k_ctl),model,'ideal');
        
        x_bar(:,k_ctl) = x_fmu(:,k_fmu);
        u_bar(:,k_ctl) = u_cmd;
        
        del_x = x_bar(:,k_ctl) - x_wp;
        J = J + 0.5*(del_x'*Q_t*del_x + u_cmd'*R*u_cmd);
        
        k_ctl = k_ctl + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model  
    FT_ext = zeros(6,1);
    x_fmu(:,k_fmu+1) = quadcopter(x_fmu(:,k_fmu),m_cmd,model,FT_ext,'fmu');
end

% Terminal Case
x_bar(:,end) = x_fmu(:,end);
del_x = x_bar(:,end) - x_wp;
J = J + 0.5* del_x'*Q_f*del_x;
disp(['[ilq_fp]: Current Cost: ',num2str(J)]);

end
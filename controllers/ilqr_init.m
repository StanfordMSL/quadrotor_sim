function nom = ilqr_init(t_fc,x_fc,wp,model)

% Determine Total Number of Time Steps
N = wp.tf*model.fc_hz+1;

nom.t_bar = linspace(t_fc,t_fc+wp.tf,N);
nom.x_bar = zeros(12,N);
nom.x_bar(:,1) = x_fc;

nom.u_bar = model.hover_wrench.*ones(4,N-1);

for k = 1:N-1
    FT_ext = zeros(6,1);
    m_cmd  = wrench2m_cmd(nom.u_bar(:,k),model);

    nom.x_bar(:,k+1) = quadcopter(nom.x_bar(:,k),m_cmd,model,FT_ext,'fc');
end

nom.index = 1;
nom.total = N;
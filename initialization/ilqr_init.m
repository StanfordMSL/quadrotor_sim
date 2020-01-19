function nom = ilqr_init(t_now,x_now,wp,fc,model)

% Determine Total Number of Time Steps
N = wp.tf*model.con_hz+1;

% Initialized Time and Trajectory and Input Variables
nom.t_bar = linspace(t_now,t_now+wp.tf,N);
nom.x_bar = zeros(13,N);
nom.x_bar(:,1) = x_now;
nom.u_bar = model.hover_wrench.*ones(4,N-1);

for k = 1:N-1
    FT_ext = zeros(6,1);
    m_cmd  = wrench2m_controller(nom.u_bar(:,k),model);

    nom.x_bar(:,k+1) = quadcopter(nom.x_bar(:,k),m_cmd,model,FT_ext,'fc');
end

% Initialize the Control Law Feedforward and Feedback Variables
nom.l = zeros(4,1,N-1);
nom.L = zeros(4,13,N-1);

nom.alpha = 0.5;
% Initialize the Cost Variable
% nom.cost = -1;

% Initialize Counter Variables (for convenience)
nom.index = 1;
nom.total = N;

% Run a First Pass of the iLQR
nom = ilqr_x(t_now,x_now,wp,nom,fc,model);
% fast_plot(nom.x_bar);
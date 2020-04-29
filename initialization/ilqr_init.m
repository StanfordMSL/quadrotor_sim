function nom = ilqr_init(wp,wts,model)

% Determine Total Number of Time Steps
N = int64(wp.tf*model.hz_ctl+1);

% Initialized Time and Trajectory and Input Variables
nom.t_bar = linspace(0,wp.tf,N);
nom.x_bar = zeros(13,N);
nom.x_bar(:,1) = wp.x(:,1);
wrench = [model.m_est*model.g ; 0 ; 0 ; 0];
nom.u_bar = wrench.*ones(4,N-1);

nom.l = zeros(4,1,N-1);
nom.L = zeros(4,13,N-1);

nom.total = N;
nom.alpha = 1;
nom.wp_curr = 1;

wp_fr = zeros(1,size(wp.t,2));
for k = 1:size(wp.t,2)
    wp_fr(1,k) = (wp.t(k)*model.hz_ctl) + 1;
end
nom.wp_fr = int64(wp_fr);

for k = 1:size(wp.t,2)-1
    x_now = wp.x(:,k);
    for j = int16(wp_fr(k)):int16(wp_fr(end))
        nom.x_bar(:,j) = wp.x(:,k+1);
    end
    nom = ilqr_x(1,x_now,wp,nom,wts,model);
end

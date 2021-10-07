function traj_plot(X,sol)

N = size(X,2);

x_obs = -1:0.01:1;
y_obs_t = sqrt(1-x_obs.^2);
y_obs_b = -sqrt(1-x_obs.^2);
figure
daspect([1 1 1])
hold on
plot(sol.value(X(1,:)),sol.value(X(2,:)));
plot(x_obs,y_obs_t,'k','linewidth',2);
plot(x_obs,y_obs_b,'k','linewidth',2);

for k = 1:3:N
    [x_arrow, y_arrow] = frame_builder(sol.value(X(:,k)));

    x = [x_arrow(1,:) ; y_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:)]';

    h_fr = plot(x,y,'linewidth',2);

    % Set the Correct Colors
    h_fr(1).Color = [1 0 0];
    h_fr(2).Color = [0 1 0];
    hold on
end

xlim([-2.5 2.5]);
ylim([0.0 1.5]);

end
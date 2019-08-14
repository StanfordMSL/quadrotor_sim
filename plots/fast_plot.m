function fast_plot(x)

figure(1)
clf
plot3(x(1,:),x(2,:),x(3,:));
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
grid on
xlim([-8.1 8.1]);
ylim([-3.2 3.2]);
zlim([0 3]);
daspect([1 1 1])
view(-50,10);

end

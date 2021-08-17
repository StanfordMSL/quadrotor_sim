function sysID_plot(log,T,X,U)

figure(2)
clf

subplot(4,2,1)
plot(T,X(1,:));
hold on
plot(log.t_fmu,log.x_fmu(1,:));
ylim([-5 5])
legend('actual','simulated')
title('pos_x');

subplot(4,2,3)
plot(T,X(2,:));
hold on
plot(log.t_fmu,log.x_fmu(2,:));
ylim([-5 5])
legend('actual','simulated')
title('pos_y');

subplot(4,2,5)
plot(T,X(3,:));
hold on
plot(log.t_fmu,log.x_fmu(3,:));
ylim([-1 3])
legend('actual','simulated')
title('pos_z');

subplot(4,2,7)
plot(T,U(1,:));
ylim([0 1]);
title('Normalized Thrust');

subplot(4,2,2)
plot(T,X(7,:));
hold on
plot(log.t_fmu,log.x_fmu(7,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q0');

subplot(4,2,4)
plot(T,X(8,:));
hold on
plot(log.t_fmu,log.x_fmu(8,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q1');

subplot(4,2,6)
plot(T,X(9,:));
hold on
plot(log.t_fmu,log.x_fmu(9,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q2');

subplot(4,2,8)
plot(T,X(10,:));
hold on
plot(log.t_fmu,log.x_fmu(10,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q3');

figure(3)
clf

subplot(3,1,1)
plot(T,U(2,:));
hold on
plot(log.t_fmu,log.x_fmu(11,:));
ylim([-5 5])
legend('actual','simulated')
title('w_x');

subplot(3,1,2)
plot(T,U(3,:));
hold on
plot(log.t_fmu,log.x_fmu(12,:));
ylim([-5 5])
legend('actual','simulated')
title('w_y');

subplot(3,1,3)
plot(T,U(4,:));
hold on
plot(log.t_fmu,log.x_fmu(13,:));
ylim([-1 3])
legend('actual','simulated')
title('w_z');
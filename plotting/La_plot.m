function La_plot(La_p,La_c)

figure(5)
clf

subplot(5,1,1)
plot(La_c.x_o)
hold on
plot(La_p.x_o,'--')
title("x_o");
legend('Current','Previous');

subplot(5,1,2)
plot(La_c.u_o)
hold on
plot(La_p.u_o,'--')
title("u_o");
legend('Current','Previous');

subplot(5,1,3)
plot(La_c.x_c)
hold on
plot(La_p.x_c,'--')
title("x_c");
legend('Current','Previous');

subplot(5,1,4)
plot(La_c.u_c)
hold on
plot(La_p.u_c,'--')
title("u_c");
legend('Current','Previous');

subplot(5,1,5)
plot(La_c.xup(1,:))
hold on
plot(La_c.xup(2,:))
plot(La_p.xup(1,:),'--')
plot(La_p.xup(2,:),'--')
title("xup");
legend('Current(1)','Current(2)','Previous(1)','Previous(2)');
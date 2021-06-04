function La_plot(La_p,La_c)

figure(5)
clf

subplot(3,1,1)
plot(La_c.objs)
hold on
plot(La_p.objs)
title("J_{obj}");
legend('Current','Previous');

subplot(3,1,2)
plot(La_c.cons)
hold on
plot(La_p.cons)
title("J_{con}");
legend('Current','Previous');

subplot(3,1,3)
plot(La_c.tots)
hold on
plot(La_p.tots)
title("J_{tot}");
legend('Current','Previous');
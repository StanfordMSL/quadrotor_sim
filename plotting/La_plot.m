function La_plot(La_p,La_c)

figure(5)
clf

c_objs = La_c.x_o + La_c.u_o + La_c.xup(1,:) +  La_c.xup(2,:);
c_cons = La_c.x_c + La_c.u_c;

p_objs = La_p.x_o + La_p.u_o + La_p.xup(1,:) +  La_p.xup(2,:);
p_cons = La_p.x_c + La_p.u_c;

subplot(3,1,1)
plot(c_objs)
hold on
plot(p_objs)
title("J_{obj}");
legend('Current','Previous');

subplot(3,1,2)
plot(c_cons)
hold on
plot(p_cons)
title("J_{con}");
legend('Current','Previous');

subplot(3,1,3)
plot(c_objs+c_cons)
hold on
plot(p_objs+p_cons)
title("J_{tot}");
legend('Current','Previous');
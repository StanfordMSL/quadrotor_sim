function u_m = w2m(in1)
%W2M
%    U_M = W2M(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Aug-2021 18:53:48

wrench1 = in1(1,:);
wrench2 = in1(2,:);
wrench3 = in1(3,:);
wrench4 = in1(4,:);
t2 = wrench1./4.0;
t4 = wrench2.*(2.5e+1./6.0);
t5 = wrench3.*(2.5e+1./6.0);
t8 = wrench4.*1.592356687898089e+1;
t3 = -t2;
t6 = -t4;
t7 = -t5;
t9 = -t8;
t10 = t3+t4+t5+t8;
t11 = t2+t5+t6+t8;
t12 = t2+t4+t7+t8;
t13 = t2+t4+t5+t9;
u_m = [-sign(t10).*sqrt(abs(t10).*4.878048780487805e+6);sign(t13).*sqrt(abs(t13).*4.878048780487805e+6);sign(t12).*sqrt(abs(t12).*4.878048780487805e+6);sign(t11).*sqrt(abs(t11).*4.878048780487805e+6)];

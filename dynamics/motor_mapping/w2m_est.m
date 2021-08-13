function u_m = w2m_est(in1)
%W2M_EST
%    U_M = W2M_EST(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Aug-2021 21:30:36

wrench1 = in1(1,:);
wrench2 = in1(2,:);
wrench3 = in1(3,:);
wrench4 = in1(4,:);
t2 = wrench1./4.0;
t4 = wrench2.*(2.5e+1./6.0);
t5 = wrench3.*(2.5e+1./6.0);
t6 = wrench4.*(2.5e+1./6.0);
t3 = -t2;
t7 = -t4;
t8 = -t5;
t9 = -t6;
t10 = t3+t4+t5+t6;
t11 = t2+t5+t6+t7;
t12 = t2+t4+t6+t8;
t13 = t2+t4+t5+t9;
u_m = [-sign(t10).*sqrt(abs(t10).*4.329004329004329e+6);sign(t13).*sqrt(abs(t13).*4.329004329004329e+6);sign(t12).*sqrt(abs(t12).*4.329004329004329e+6);sign(t11).*sqrt(abs(t11).*4.329004329004329e+6)];

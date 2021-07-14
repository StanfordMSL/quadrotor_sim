function u_m = w2m_est(in1)
%W2M_EST
%    U_M = W2M_EST(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    14-Jul-2021 15:43:26

wrench1 = in1(1,:);
wrench2 = in1(2,:);
wrench3 = in1(3,:);
wrench4 = in1(4,:);
t2 = wrench1.*6.839571022105493e+5;
t3 = wrench4.*6.839571022105493e+6;
t5 = wrench2.*7.728328838537281e+6;
t6 = wrench3.*7.728328838537281e+6;
t4 = -t3;
t7 = -t5;
t8 = -t6;
u_m = [sqrt(t2+t4+t7+t8);sqrt(t2+t4+t5+t6);sqrt(t2+t3+t5+t8);sqrt(t2+t3+t6+t7)];

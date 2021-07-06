function u_m = w2m_act(in1)
%W2M_ACT
%    U_M = W2M_ACT(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Jul-2021 15:08:45

wrench1 = in1(1,:);
wrench2 = in1(2,:);
wrench3 = in1(3,:);
wrench4 = in1(4,:);
t2 = wrench1.*4.280821917808219e+4;
t3 = wrench4.*7.134703196347032e+5;
t5 = wrench2.*4.837086912777649e+5;
t6 = wrench3.*4.837086912777649e+5;
t4 = -t3;
t7 = -t5;
t8 = -t6;
u_m = [sqrt(t2+t4+t7+t8);sqrt(t2+t4+t5+t6);sqrt(t2+t3+t5+t8);sqrt(t2+t3+t6+t7)];

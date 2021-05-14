function con = gate_con(in1,in2)
%GATE_CON
%    CON = GATE_CON(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    13-May-2021 22:15:34

x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = x7.^2;
t3 = x8.^2;
t4 = x9.^2;
t5 = x10.^2;
t6 = x7.*x10.*1.250962292533972;
t7 = x8.*x9.*1.250962292533972;
t8 = x7.*x8.*6.254811462669859e-1;
t9 = x7.*x9.*6.254811462669859e-1;
t12 = x8.*x10.*6.254811462669859e-1;
t13 = x9.*x10.*6.254811462669859e-1;
t14 = x7.*x8.*1.252195409134752;
t15 = x7.*x9.*1.252195409134752;
t16 = x7.*x10.*6.260977045673759e-1;
t17 = x8.*x9.*6.260977045673759e-1;
t18 = x8.*x10.*1.252195409134752;
t19 = x9.*x10.*1.252195409134752;
t10 = -t6;
t11 = -t7;
t20 = t2.*6.254811462669859e-1;
t21 = t3.*6.254811462669859e-1;
t22 = t4.*6.254811462669859e-1;
t23 = t5.*6.254811462669859e-1;
t24 = -t8;
t25 = -t9;
t26 = -t12;
t27 = -t13;
t28 = t2.*3.130488522836879e-1;
t29 = t3.*3.130488522836879e-1;
t30 = t4.*3.130488522836879e-1;
t31 = t5.*3.130488522836879e-1;
t32 = -t14;
t33 = -t15;
t34 = -t16;
t35 = -t17;
t36 = -t18;
t37 = -t19;
t38 = -t20;
t39 = -t21;
t40 = -t22;
t41 = -t23;
t42 = -t28;
t43 = -t29;
t44 = -t30;
t45 = -t31;
con = [t9+t16+t17+t26+6.571066953866451;t8+t13+t29+t31+t42+t44+6.571066953866451;t12+t25+t34+t35+6.571066953866451;t24+t27+t28+t30+t43+t45+6.571066953866451;t10+t11+t15+t36-4.930332395436155e-1;t14+t19+t20+t22+t39+t41-4.930332395436155e-1;t6+t7+t18+t33-4.930332395436155e-1;t21+t23+t32+t37+t38+t40-4.930332395436155e-1;t12+t25+t34+t35-7.571066953866451;t24+t27+t28+t30+t43+t45-7.571066953866451;t9+t16+t17+t26-7.571066953866451;t8+t13+t29+t31+t42+t44-7.571066953866451;t6+t7+t18+t33-5.069667604563845e-1;t21+t23+t32+t37+t38+t40-5.069667604563845e-1;t10+t11+t15+t36-5.069667604563845e-1;t14+t19+t20+t22+t39+t41-5.069667604563845e-1];

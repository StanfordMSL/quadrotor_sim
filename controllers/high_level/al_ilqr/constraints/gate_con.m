function con = gate_con(in1,in2)
%GATE_CON
%    CON = GATE_CON(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    04-Jun-2021 16:02:26

x2 = in1(2,:);
x3 = in1(3,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = x7.^2;
t3 = x8.^2;
t4 = x9.^2;
t5 = x10.^2;
t6 = x3.*3.043466085251945;
t7 = x2.*1.521733042625967;
t9 = x2.*9.525613585902026;
t10 = x3.*4.762806792951013;
t14 = x7.*x8.*5.386934970895942e-1;
t15 = x7.*x9.*5.386934970895942e-1;
t16 = x8.*x10.*5.386934970895942e-1;
t17 = x9.*x10.*5.386934970895942e-1;
t18 = x7.*x10.*1.686033604704659;
t19 = x8.*x9.*1.686033604704659;
t24 = x7.*x10.*2.693467485447961e-1;
t25 = x8.*x9.*2.693467485447961e-1;
t26 = x7.*x8.*8.430168023523293e-1;
t27 = x7.*x9.*8.430168023523293e-1;
t30 = x8.*x10.*8.430168023523293e-1;
t31 = x9.*x10.*8.430168023523293e-1;
t8 = -t6;
t11 = -t7;
t12 = -t9;
t13 = -t10;
t20 = -t14;
t21 = -t15;
t22 = -t16;
t23 = -t17;
t28 = -t18;
t29 = -t19;
t32 = t2.*1.346733742723981e-1;
t33 = t3.*1.346733742723981e-1;
t34 = t4.*1.346733742723981e-1;
t35 = t5.*1.346733742723981e-1;
t36 = -t24;
t37 = -t25;
t38 = -t26;
t39 = -t27;
t40 = -t30;
t41 = -t31;
t46 = t2.*8.430168023523293e-1;
t47 = t3.*8.430168023523293e-1;
t48 = t4.*8.430168023523293e-1;
t49 = t5.*8.430168023523293e-1;
t42 = -t32;
t43 = -t33;
t44 = -t34;
t45 = -t35;
t50 = -t46;
t51 = -t47;
t52 = -t48;
t53 = -t49;
con = [t7+t10+t24+t25+t30+t39-3.741073750325046;t7+t10+t33+t35+t38+t41+t42+t44-3.741073750325046;t7+t10+t27+t36+t37+t40-3.741073750325046;t7+t10+t26+t31+t32+t34+t43+t45-3.741073750325046;t8+t9+t15+t18+t19+t22+1.206907967115397e+1;t8+t9+t14+t17+t47+t49+t50+t52+1.206907967115397e+1;t8+t9+t16+t21+t28+t29+1.206907967115397e+1;t8+t9+t20+t23+t46+t48+t51+t53+1.206907967115397e+1;t11+t13+t27+t36+t37+t40+2.741073750325046;t11+t13+t26+t31+t32+t34+t43+t45+2.741073750325046;t11+t13+t24+t25+t30+t39+2.741073750325046;t11+t13+t33+t35+t38+t41+t42+t44+2.741073750325046;t6+t12+t16+t21+t28+t29-1.306907967115397e+1;t6+t12+t20+t23+t46+t48+t51+t53-1.306907967115397e+1;t6+t12+t15+t18+t19+t22-1.306907967115397e+1;t6+t12+t14+t17+t47+t49+t50+t52-1.306907967115397e+1];

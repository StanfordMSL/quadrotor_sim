function con = gate_con(in1,in2)
%GATE_CON
%    CON = GATE_CON(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Jul-2021 15:09:01

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
t6 = x2./2.0e+1;
t7 = x3./2.0e+1;
t10 = x7.*x8.*8.85e-3;
t11 = x7.*x9.*8.85e-3;
t12 = x7.*x10.*8.85e-3;
t13 = x8.*x9.*8.85e-3;
t14 = x8.*x10.*8.85e-3;
t15 = x9.*x10.*8.85e-3;
t8 = -t6;
t9 = -t7;
t16 = t2.*4.425e-3;
t17 = t3.*4.425e-3;
t18 = t4.*4.425e-3;
t19 = t5.*4.425e-3;
t20 = -t10;
t21 = -t11;
t22 = -t12;
t23 = -t13;
t24 = -t14;
t25 = -t15;
t26 = -t16;
t27 = -t17;
t28 = -t18;
t29 = -t19;
con = [t9+t11+t24-9.0./2.0e+1;t9+t10+t15-9.0./2.0e+1;t9+t14+t21-9.0./2.0e+1;t9+t20+t25-9.0./2.0e+1;t8+t22+t23-1.0./2.0;t8+t16+t18+t27+t29-1.0./2.0;t8+t12+t13-1.0./2.0;t8+t17+t19+t26+t28-1.0./2.0;t7+t14+t21-1.1e+1./2.0e+1;t7+t20+t25-1.1e+1./2.0e+1;t7+t11+t24-1.1e+1./2.0e+1;t7+t10+t15-1.1e+1./2.0e+1;t6+t12+t13-1.0./2.0;t6+t17+t19+t26+t28-1.0./2.0;t6+t22+t23-1.0./2.0;t6+t16+t18+t27+t29-1.0./2.0];

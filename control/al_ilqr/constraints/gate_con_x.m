function con_x = gate_con_x(in1,in2)
%GATE_CON_X
%    CON_X = GATE_CON_X(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Jun-2021 15:55:04

x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = x7.*8.85e-3;
t3 = x8.*8.85e-3;
t4 = x9.*8.85e-3;
t5 = x10.*8.85e-3;
t6 = -t2;
t7 = -t3;
t8 = -t4;
t9 = -t5;
con_x = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0e+1,-1.0./2.0e+1,-1.0./2.0e+1,-1.0./2.0e+1,0.0,0.0,0.0,0.0,1.0./2.0e+1,1.0./2.0e+1,1.0./2.0e+1,1.0./2.0e+1,-1.0./2.0e+1,-1.0./2.0e+1,-1.0./2.0e+1,-1.0./2.0e+1,0.0,0.0,0.0,0.0,1.0./2.0e+1,1.0./2.0e+1,1.0./2.0e+1,1.0./2.0e+1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t4,t3,t8,t7,t9,t2,t5,t6,t8,t7,t4,t3,t5,t6,t9,t2,t9,t2,t5,t6,t8,t7,t4,t3,t5,t6,t9,t2,t4,t3,t8,t7,t2,t5,t6,t9,t7,t4,t3,t8,t6,t9,t2,t5,t3,t8,t7,t4,t7,t4,t3,t8,t6,t9,t2,t5,t3,t8,t7,t4,t2,t5,t6,t9],[16,10]);

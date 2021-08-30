function dCn_x = dCn_x(in1,in2,in3,in4,trig)
%DCN_X
%    DCN_X = DCN_X(IN1,IN2,IN3,IN4,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Aug-2021 18:55:31

Qin_d1 = in4(1,:);
Qin_d2 = in4(2,:);
Qin_d3 = in4(3,:);
Qin_d4 = in4(4,:);
Qin_d5 = in4(5,:);
Qin_d6 = in4(6,:);
Qin_d7 = in4(7,:);
Qin_d8 = in4(8,:);
Qin_d9 = in4(9,:);
Qin_d10 = in4(10,:);
Qin_d11 = in4(11,:);
Qin_d12 = in4(12,:);
Qin_d13 = in4(13,:);
Qin_d14 = in4(14,:);
Qin_d15 = in4(15,:);
Qin_d16 = in4(16,:);
Qin_d17 = in4(17,:);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
x11 = in1(11,:);
x12 = in1(12,:);
x13 = in1(13,:);
x14 = in1(14,:);
x15 = in1(15,:);
x16 = in1(16,:);
x17 = in1(17,:);
x_bar1 = in2(1,:);
x_bar2 = in2(2,:);
x_bar3 = in2(3,:);
x_bar4 = in2(4,:);
x_bar5 = in2(5,:);
x_bar6 = in2(6,:);
x_bar7 = in2(7,:);
x_bar8 = in2(8,:);
x_bar9 = in2(9,:);
x_bar10 = in2(10,:);
x_bar11 = in2(11,:);
x_bar12 = in2(12,:);
x_bar13 = in2(13,:);
x_bar14 = in2(14,:);
x_bar15 = in2(15,:);
x_bar16 = in2(16,:);
x_bar17 = in2(17,:);
x_star1 = in3(1,:);
x_star2 = in3(2,:);
x_star3 = in3(3,:);
x_star4 = in3(4,:);
x_star5 = in3(5,:);
x_star6 = in3(6,:);
x_star7 = in3(7,:);
x_star8 = in3(8,:);
x_star9 = in3(9,:);
x_star10 = in3(10,:);
x_star11 = in3(11,:);
x_star12 = in3(12,:);
x_star13 = in3(13,:);
x_star14 = in3(14,:);
x_star15 = in3(15,:);
x_star16 = in3(16,:);
x_star17 = in3(17,:);
t2 = -x_bar1;
t3 = -x_bar2;
t4 = -x_bar3;
t5 = -x_bar4;
t6 = -x_bar5;
t7 = -x_bar6;
t8 = -x_bar7;
t9 = -x_bar8;
t10 = -x_bar9;
t11 = -x_bar10;
t12 = -x_bar11;
t13 = -x_bar12;
t14 = -x_bar13;
t15 = -x_bar14;
t16 = -x_bar15;
t17 = -x_bar16;
t18 = -x_bar17;
t19 = t2+x1;
t20 = t3+x2;
t21 = t4+x3;
t22 = t5+x4;
t23 = t6+x5;
t24 = t7+x6;
t25 = t8+x7;
t26 = t9+x8;
t27 = t10+x9;
t28 = t11+x10;
t29 = t12+x11;
t30 = t13+x12;
t31 = t14+x13;
t32 = t15+x14;
t33 = t16+x15;
t34 = t17+x16;
t35 = t18+x17;
dCn_x = trig.*(Qin_d1.*t19.^2+Qin_d2.*t20.^2+Qin_d3.*t21.^2+Qin_d4.*t22.^2+Qin_d5.*t23.^2+Qin_d6.*t24.^2+Qin_d7.*t25.^2+Qin_d8.*t26.^2+Qin_d9.*t27.^2+Qin_d10.*t28.^2+Qin_d11.*t29.^2+Qin_d12.*t30.^2+Qin_d13.*t31.^2+Qin_d14.*t32.^2+Qin_d15.*t33.^2+Qin_d16.*t34.^2+Qin_d17.*t35.^2+Qin_d1.*t19.*(x_bar1-x_star1)+Qin_d2.*t20.*(x_bar2-x_star2)+Qin_d3.*t21.*(x_bar3-x_star3)+Qin_d4.*t22.*(x_bar4-x_star4)+Qin_d5.*t23.*(x_bar5-x_star5)+Qin_d6.*t24.*(x_bar6-x_star6)+Qin_d7.*t25.*(x_bar7-x_star7)+Qin_d8.*t26.*(x_bar8-x_star8)+Qin_d9.*t27.*(x_bar9-x_star9)+Qin_d10.*t28.*(x_bar10-x_star10)+Qin_d11.*t29.*(x_bar11-x_star11)+Qin_d12.*t30.*(x_bar12-x_star12)+Qin_d13.*t31.*(x_bar13-x_star13)+Qin_d14.*t32.*(x_bar14-x_star14)+Qin_d15.*t33.*(x_bar15-x_star15)+Qin_d16.*t34.*(x_bar16-x_star16)+Qin_d17.*t35.*(x_bar17-x_star17));

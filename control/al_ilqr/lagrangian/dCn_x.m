function dCn_x = dCn_x(in1,in2,in3,in4,trig)
%DCN_X
%    DCN_X = DCN_X(IN1,IN2,IN3,IN4,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    23-Aug-2021 13:01:31

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
t12 = t2+x1;
t13 = t3+x2;
t14 = t4+x3;
t15 = t5+x4;
t16 = t6+x5;
t17 = t7+x6;
t18 = t8+x7;
t19 = t9+x8;
t20 = t10+x9;
t21 = t11+x10;
dCn_x = trig.*(Qin_d1.*t12.^2+Qin_d2.*t13.^2+Qin_d3.*t14.^2+Qin_d4.*t15.^2+Qin_d5.*t16.^2+Qin_d6.*t17.^2+Qin_d7.*t18.^2+Qin_d8.*t19.^2+Qin_d9.*t20.^2+Qin_d10.*t21.^2+Qin_d1.*t12.*(x_bar1-x_star1)+Qin_d2.*t13.*(x_bar2-x_star2)+Qin_d3.*t14.*(x_bar3-x_star3)+Qin_d4.*t15.*(x_bar4-x_star4)+Qin_d5.*t16.*(x_bar5-x_star5)+Qin_d6.*t17.*(x_bar6-x_star6)+Qin_d7.*t18.*(x_bar7-x_star7)+Qin_d8.*t19.*(x_bar8-x_star8)+Qin_d9.*t20.*(x_bar9-x_star9)+Qin_d10.*t21.*(x_bar10-x_star10));

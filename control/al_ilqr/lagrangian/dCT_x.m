function dCT_x = dCT_x(in1,in2,in3,trig)
%DCT_X
%    DCT_X = DCT_X(IN1,IN2,IN3,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Jul-2021 17:53:58

x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x_bar1 = in2(1,:);
x_bar2 = in2(2,:);
x_bar3 = in2(3,:);
x_bar4 = in2(4,:);
x_bar5 = in2(5,:);
x_bar6 = in2(6,:);
x_star1 = in3(1,:);
x_star2 = in3(2,:);
x_star3 = in3(3,:);
x_star4 = in3(4,:);
x_star5 = in3(5,:);
x_star6 = in3(6,:);
t2 = -x_bar1;
t3 = -x_bar2;
t4 = -x_bar3;
t5 = -x_bar4;
t6 = -x_bar5;
t7 = -x_bar6;
t8 = t2+x1;
t9 = t3+x2;
t10 = t4+x3;
t11 = t5+x4;
t12 = t6+x5;
t13 = t7+x6;
dCT_x = trig.*(t8.*(x_bar1-x_star1)+t9.*(x_bar2-x_star2)+t10.*(x_bar3-x_star3)+t11.*(x_bar4-x_star4)+t12.*(x_bar5-x_star5)+t13.*(x_bar6-x_star6)+t8.^2+t9.^2+t10.^2+t11.^2+t12.^2+t13.^2);

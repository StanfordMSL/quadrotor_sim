function obj_cost_N = obj_cost_N(in1,in2,in3,in4,in5,in6)
%OBJ_COST_N
%    OBJ_COST_N = OBJ_COST_N(IN1,IN2,IN3,IN4,IN5,IN6)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    14-Jul-2021 15:43:33

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
x_bar1 = in3(1,:);
x_bar2 = in3(2,:);
x_bar3 = in3(3,:);
x_bar4 = in3(4,:);
x_bar5 = in3(5,:);
x_bar6 = in3(6,:);
x_bar7 = in3(7,:);
x_bar8 = in3(8,:);
x_bar9 = in3(9,:);
x_bar10 = in3(10,:);
x_star1 = in5(1,:);
x_star2 = in5(2,:);
x_star3 = in5(3,:);
x_star4 = in5(4,:);
x_star5 = in5(5,:);
x_star6 = in5(6,:);
x_star7 = in5(7,:);
x_star8 = in5(8,:);
x_star9 = in5(9,:);
x_star10 = in5(10,:);
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
t12 = -x_star1;
t13 = -x_star2;
t14 = -x_star3;
t15 = -x_star4;
t16 = -x_star5;
t17 = -x_star6;
t18 = -x_star7;
t19 = -x_star8;
t20 = -x_star9;
t21 = -x_star10;
t22 = t2+x1;
t23 = t3+x2;
t24 = t4+x3;
t25 = t5+x4;
t26 = t6+x5;
t27 = t7+x6;
t28 = t8+x7;
t29 = t9+x8;
t30 = t10+x9;
t31 = t11+x10;
t32 = t12+x_bar1;
t33 = t13+x_bar2;
t34 = t14+x_bar3;
t35 = t15+x_bar4;
t36 = t16+x_bar5;
t37 = t17+x_bar6;
t38 = t18+x_bar7;
t39 = t19+x_bar8;
t40 = t20+x_bar9;
t41 = t21+x_bar10;
obj_cost_N = t22.*t32+t23.*t33+t24.*t34+t25.*t35+t26.*t36+t27.*t37+t28.*t38+t29.*t39+t30.*t40+t31.*t41+t22.^2+t23.^2+t24.^2+t25.^2+t26.^2+t27.^2+t28.^2+t29.^2+t30.^2+t31.^2+t32.*(x_bar1./2.0-x_star1./2.0)+t33.*(x_bar2./2.0-x_star2./2.0)+t34.*(x_bar3./2.0-x_star3./2.0)+t35.*(x_bar4./2.0-x_star4./2.0)+t36.*(x_bar5./2.0-x_star5./2.0)+t37.*(x_bar6./2.0-x_star6./2.0)+t38.*(x_bar7./2.0-x_star7./2.0)+t39.*(x_bar8./2.0-x_star8./2.0)+t40.*(x_bar9./2.0-x_star9./2.0)+t41.*(x_bar10./2.0-x_star10./2.0);

function dqT = dqT_calc(in1,in2,trig)
%DQT_CALC
%    DQT = DQT_CALC(IN1,IN2,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    22-Jul-2021 14:19:18

x_bar1 = in1(1,:);
x_bar2 = in1(2,:);
x_bar3 = in1(3,:);
x_bar4 = in1(4,:);
x_bar5 = in1(5,:);
x_bar6 = in1(6,:);
x_bar7 = in1(7,:);
x_bar8 = in1(8,:);
x_bar9 = in1(9,:);
x_bar10 = in1(10,:);
x_star1 = in2(1,:);
x_star2 = in2(2,:);
x_star3 = in2(3,:);
x_star4 = in2(4,:);
x_star5 = in2(5,:);
x_star6 = in2(6,:);
x_star7 = in2(7,:);
x_star8 = in2(8,:);
x_star9 = in2(9,:);
x_star10 = in2(10,:);
dqT = [x_bar1-x_star1;x_bar2-x_star2;x_bar3-x_star3;x_bar4-x_star4;x_bar5-x_star5;x_bar6-x_star6;x_bar7-x_star7;x_bar8-x_star8;x_bar9-x_star9;x_bar10-x_star10];

function dCN_p = dCN_p(in1,in2,in3,trig)
%DCN_P
%    DCN_P = DCN_P(IN1,IN2,IN3,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    25-Aug-2021 14:57:25

Qin_d1 = in3(1,:);
Qin_d2 = in3(2,:);
Qin_d3 = in3(3,:);
Qin_d4 = in3(4,:);
Qin_d5 = in3(5,:);
Qin_d6 = in3(6,:);
Qin_d7 = in3(7,:);
Qin_d8 = in3(8,:);
Qin_d9 = in3(9,:);
Qin_d10 = in3(10,:);
Qin_d11 = in3(11,:);
Qin_d12 = in3(12,:);
Qin_d13 = in3(13,:);
Qin_d14 = in3(14,:);
Qin_d15 = in3(15,:);
Qin_d16 = in3(16,:);
Qin_d17 = in3(17,:);
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
x_bar11 = in1(11,:);
x_bar12 = in1(12,:);
x_bar13 = in1(13,:);
x_bar14 = in1(14,:);
x_bar15 = in1(15,:);
x_bar16 = in1(16,:);
x_bar17 = in1(17,:);
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
x_star11 = in2(11,:);
x_star12 = in2(12,:);
x_star13 = in2(13,:);
x_star14 = in2(14,:);
x_star15 = in2(15,:);
x_star16 = in2(16,:);
x_star17 = in2(17,:);
dCN_p = trig.*(Qin_d1.*(x_bar1./2.0-x_star1./2.0).*(x_bar1-x_star1)+Qin_d2.*(x_bar2./2.0-x_star2./2.0).*(x_bar2-x_star2)+Qin_d3.*(x_bar3./2.0-x_star3./2.0).*(x_bar3-x_star3)+Qin_d4.*(x_bar4./2.0-x_star4./2.0).*(x_bar4-x_star4)+Qin_d5.*(x_bar5./2.0-x_star5./2.0).*(x_bar5-x_star5)+Qin_d6.*(x_bar6./2.0-x_star6./2.0).*(x_bar6-x_star6)+Qin_d7.*(x_bar7./2.0-x_star7./2.0).*(x_bar7-x_star7)+Qin_d8.*(x_bar8./2.0-x_star8./2.0).*(x_bar8-x_star8)+Qin_d9.*(x_bar9./2.0-x_star9./2.0).*(x_bar9-x_star9)+Qin_d10.*(x_bar10./2.0-x_star10./2.0).*(x_bar10-x_star10)+Qin_d11.*(x_bar11./2.0-x_star11./2.0).*(x_bar11-x_star11)+Qin_d12.*(x_bar12./2.0-x_star12./2.0).*(x_bar12-x_star12)+Qin_d13.*(x_bar13./2.0-x_star13./2.0).*(x_bar13-x_star13)+Qin_d14.*(x_bar14./2.0-x_star14./2.0).*(x_bar14-x_star14)+Qin_d15.*(x_bar15./2.0-x_star15./2.0).*(x_bar15-x_star15)+Qin_d16.*(x_bar16./2.0-x_star16./2.0).*(x_bar16-x_star16)+Qin_d17.*(x_bar17./2.0-x_star17./2.0).*(x_bar17-x_star17));

function dCn_p = dCn_p(in1,in2,in3,in4,in5,in6,trig)
%DCN_P
%    DCN_P = DCN_P(IN1,IN2,IN3,IN4,IN5,IN6,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    26-Aug-2021 19:42:49

Qin_d1 = in5(1,:);
Qin_d2 = in5(2,:);
Qin_d3 = in5(3,:);
Qin_d4 = in5(4,:);
Qin_d5 = in5(5,:);
Qin_d6 = in5(6,:);
Qin_d7 = in5(7,:);
Qin_d8 = in5(8,:);
Qin_d9 = in5(9,:);
Qin_d10 = in5(10,:);
Qin_d11 = in5(11,:);
Qin_d12 = in5(12,:);
Qin_d13 = in5(13,:);
Qin_d14 = in5(14,:);
Qin_d15 = in5(15,:);
Qin_d16 = in5(16,:);
Qin_d17 = in5(17,:);
Rin_d1 = in6(1,:);
Rin_d2 = in6(2,:);
Rin_d3 = in6(3,:);
Rin_d4 = in6(4,:);
u_bar1 = in2(1,:);
u_bar2 = in2(2,:);
u_bar3 = in2(3,:);
u_bar4 = in2(4,:);
u_star1 = in4(1,:);
u_star2 = in4(2,:);
u_star3 = in4(3,:);
u_star4 = in4(4,:);
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
dCn_p = trig.*(Rin_d1.*(u_bar1./2.0-u_star1./2.0).*(u_bar1-u_star1)+Rin_d2.*(u_bar2./2.0-u_star2./2.0).*(u_bar2-u_star2)+Rin_d3.*(u_bar3./2.0-u_star3./2.0).*(u_bar3-u_star3)+Rin_d4.*(u_bar4./2.0-u_star4./2.0).*(u_bar4-u_star4)+Qin_d1.*(x_bar1./2.0-x_star1./2.0).*(x_bar1-x_star1)+Qin_d2.*(x_bar2./2.0-x_star2./2.0).*(x_bar2-x_star2)+Qin_d3.*(x_bar3./2.0-x_star3./2.0).*(x_bar3-x_star3)+Qin_d4.*(x_bar4./2.0-x_star4./2.0).*(x_bar4-x_star4)+Qin_d5.*(x_bar5./2.0-x_star5./2.0).*(x_bar5-x_star5)+Qin_d6.*(x_bar6./2.0-x_star6./2.0).*(x_bar6-x_star6)+Qin_d7.*(x_bar7./2.0-x_star7./2.0).*(x_bar7-x_star7)+Qin_d8.*(x_bar8./2.0-x_star8./2.0).*(x_bar8-x_star8)+Qin_d9.*(x_bar9./2.0-x_star9./2.0).*(x_bar9-x_star9)+Qin_d10.*(x_bar10./2.0-x_star10./2.0).*(x_bar10-x_star10)+Qin_d11.*(x_bar11./2.0-x_star11./2.0).*(x_bar11-x_star11)+Qin_d12.*(x_bar12./2.0-x_star12./2.0).*(x_bar12-x_star12)+Qin_d13.*(x_bar13./2.0-x_star13./2.0).*(x_bar13-x_star13)+Qin_d14.*(x_bar14./2.0-x_star14./2.0).*(x_bar14-x_star14)+Qin_d15.*(x_bar15./2.0-x_star15./2.0).*(x_bar15-x_star15)+Qin_d16.*(x_bar16./2.0-x_star16./2.0).*(x_bar16-x_star16)+Qin_d17.*(x_bar17./2.0-x_star17./2.0).*(x_bar17-x_star17));

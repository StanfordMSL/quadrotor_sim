function dCn_u = dCn_u(in1,in2,in3,in4,trig)
%DCN_U
%    DCN_U = DCN_U(IN1,IN2,IN3,IN4,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    17-Aug-2021 16:36:44

Rin_d1 = in4(1,:);
Rin_d2 = in4(2,:);
Rin_d3 = in4(3,:);
Rin_d4 = in4(4,:);
u1 = in1(1,:);
u2 = in1(2,:);
u3 = in1(3,:);
u4 = in1(4,:);
u_bar1 = in2(1,:);
u_bar2 = in2(2,:);
u_bar3 = in2(3,:);
u_bar4 = in2(4,:);
u_star1 = in3(1,:);
u_star2 = in3(2,:);
u_star3 = in3(3,:);
u_star4 = in3(4,:);
t2 = -u_bar1;
t3 = -u_bar2;
t4 = -u_bar3;
t5 = -u_bar4;
t6 = t2+u1;
t7 = t3+u2;
t8 = t4+u3;
t9 = t5+u4;
dCn_u = trig.*(Rin_d1.*t6.^2+Rin_d2.*t7.^2+Rin_d3.*t8.^2+Rin_d4.*t9.^2+Rin_d1.*t6.*(u_bar1-u_star1)+Rin_d2.*t7.*(u_bar2-u_star2)+Rin_d3.*t8.*(u_bar3-u_star3)+Rin_d4.*t9.*(u_bar4-u_star4));

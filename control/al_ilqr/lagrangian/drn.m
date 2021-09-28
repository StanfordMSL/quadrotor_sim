function drn = drn(in1,in2,in3,trig)
%DRN
%    DRN = DRN(IN1,IN2,IN3,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    27-Sep-2021 18:12:03

Rin_d1 = in3(1,:);
Rin_d2 = in3(2,:);
Rin_d3 = in3(3,:);
Rin_d4 = in3(4,:);
u_bar1 = in1(1,:);
u_bar2 = in1(2,:);
u_bar3 = in1(3,:);
u_bar4 = in1(4,:);
u_star1 = in2(1,:);
u_star2 = in2(2,:);
u_star3 = in2(3,:);
u_star4 = in2(4,:);
drn = [Rin_d1.*(u_bar1-u_star1);Rin_d2.*(u_bar2-u_star2);Rin_d3.*(u_bar3-u_star3);Rin_d4.*(u_bar4-u_star4)];

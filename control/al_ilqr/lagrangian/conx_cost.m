function conx_cost = conx_cost(in1,in2,in3)
%CONX_COST
%    CONX_COST = CONX_COST(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Aug-2021 18:55:34

conx1 = in1(1,:);
conx2 = in1(2,:);
conx3 = in1(3,:);
conx4 = in1(4,:);
conx5 = in1(5,:);
conx6 = in1(6,:);
conx7 = in1(7,:);
conx8 = in1(8,:);
conx9 = in1(9,:);
conx10 = in1(10,:);
conx11 = in1(11,:);
conx12 = in1(12,:);
conx13 = in1(13,:);
conx14 = in1(14,:);
conx15 = in1(15,:);
conx16 = in1(16,:);
lam_x1 = in2(1,:);
lam_x2 = in2(2,:);
lam_x3 = in2(3,:);
lam_x4 = in2(4,:);
lam_x5 = in2(5,:);
lam_x6 = in2(6,:);
lam_x7 = in2(7,:);
lam_x8 = in2(8,:);
lam_x9 = in2(9,:);
lam_x10 = in2(10,:);
lam_x11 = in2(11,:);
lam_x12 = in2(12,:);
lam_x13 = in2(13,:);
lam_x14 = in2(14,:);
lam_x15 = in2(15,:);
lam_x16 = in2(16,:);
mud_x1 = in3(1,:);
mud_x2 = in3(2,:);
mud_x3 = in3(3,:);
mud_x4 = in3(4,:);
mud_x5 = in3(5,:);
mud_x6 = in3(6,:);
mud_x7 = in3(7,:);
mud_x8 = in3(8,:);
mud_x9 = in3(9,:);
mud_x10 = in3(10,:);
mud_x11 = in3(11,:);
mud_x12 = in3(12,:);
mud_x13 = in3(13,:);
mud_x14 = in3(14,:);
mud_x15 = in3(15,:);
mud_x16 = in3(16,:);
conx_cost = conx1.*(lam_x1+(conx1.*mud_x1)./2.0)+conx2.*(lam_x2+(conx2.*mud_x2)./2.0)+conx3.*(lam_x3+(conx3.*mud_x3)./2.0)+conx4.*(lam_x4+(conx4.*mud_x4)./2.0)+conx5.*(lam_x5+(conx5.*mud_x5)./2.0)+conx6.*(lam_x6+(conx6.*mud_x6)./2.0)+conx7.*(lam_x7+(conx7.*mud_x7)./2.0)+conx8.*(lam_x8+(conx8.*mud_x8)./2.0)+conx9.*(lam_x9+(conx9.*mud_x9)./2.0)+conx10.*(lam_x10+(conx10.*mud_x10)./2.0)+conx11.*(lam_x11+(conx11.*mud_x11)./2.0)+conx12.*(lam_x12+(conx12.*mud_x12)./2.0)+conx13.*(lam_x13+(conx13.*mud_x13)./2.0)+conx14.*(lam_x14+(conx14.*mud_x14)./2.0)+conx15.*(lam_x15+(conx15.*mud_x15)./2.0)+conx16.*(lam_x16+(conx16.*mud_x16)./2.0);

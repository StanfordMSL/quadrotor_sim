function conu_cost = conu_cost(in1,in2,in3)
%CONU_COST
%    CONU_COST = CONU_COST(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    23-Aug-2021 21:06:52

conu1 = in1(1,:);
conu2 = in1(2,:);
conu3 = in1(3,:);
conu4 = in1(4,:);
conu5 = in1(5,:);
conu6 = in1(6,:);
conu7 = in1(7,:);
conu8 = in1(8,:);
lam_u1 = in2(1,:);
lam_u2 = in2(2,:);
lam_u3 = in2(3,:);
lam_u4 = in2(4,:);
lam_u5 = in2(5,:);
lam_u6 = in2(6,:);
lam_u7 = in2(7,:);
lam_u8 = in2(8,:);
mud_u1 = in3(1,:);
mud_u2 = in3(2,:);
mud_u3 = in3(3,:);
mud_u4 = in3(4,:);
mud_u5 = in3(5,:);
mud_u6 = in3(6,:);
mud_u7 = in3(7,:);
mud_u8 = in3(8,:);
conu_cost = conu1.*(lam_u1+(conu1.*mud_u1)./2.0)+conu2.*(lam_u2+(conu2.*mud_u2)./2.0)+conu3.*(lam_u3+(conu3.*mud_u3)./2.0)+conu4.*(lam_u4+(conu4.*mud_u4)./2.0)+conu5.*(lam_u5+(conu5.*mud_u5)./2.0)+conu6.*(lam_u6+(conu6.*mud_u6)./2.0)+conu7.*(lam_u7+(conu7.*mud_u7)./2.0)+conu8.*(lam_u8+(conu8.*mud_u8)./2.0);

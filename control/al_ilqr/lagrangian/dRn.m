function dRn = dRn(in1,trig)
%DRN
%    DRN = DRN(IN1,TRIG)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    26-Aug-2021 19:42:47

Rin_d1 = in1(1,:);
Rin_d2 = in1(2,:);
Rin_d3 = in1(3,:);
Rin_d4 = in1(4,:);
dRn = reshape([Rin_d1,0.0,0.0,0.0,0.0,Rin_d2,0.0,0.0,0.0,0.0,Rin_d3,0.0,0.0,0.0,0.0,Rin_d4],[4,4]);

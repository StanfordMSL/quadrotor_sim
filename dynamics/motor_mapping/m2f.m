function F_m = m2f(in1)
%M2F
%    F_M = M2F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Sep-2021 03:19:57

wm1 = in1(1,:);
wm2 = in1(2,:);
wm3 = in1(3,:);
wm4 = in1(4,:);
F_m = [wm1.^2.*sign(wm1).*2.05e-7;wm2.^2.*sign(wm2).*2.05e-7;wm3.^2.*sign(wm3).*2.05e-7;wm4.^2.*sign(wm4).*2.05e-7];

function H4 = H4_calc(in1,in2)
%H4_CALC
%    H4 = H4_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    14-Oct-2021 19:48:48

u1 = in2(1,:);
x3 = in1(3,:);
H4 = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,(u1.*sin(x3))./5.3e+1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,6]);

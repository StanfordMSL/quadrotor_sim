function H5 = H5_calc(in1,in2)
%H5_CALC
%    H5 = H5_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    14-Oct-2021 23:39:49

u1 = in2(1,:);
x3 = in1(3,:);
H5 = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,u1.*cos(x3).*(-1.0./5.3e+1),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,6]);

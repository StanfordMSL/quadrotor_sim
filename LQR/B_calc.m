function B = B_calc(in1,in2)
%B_CALC
%    B = B_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    14-Oct-2021 16:01:41

x3 = in1(3,:);
B = reshape([0.0,0.0,0.0,sin(x3).*(-1.0./5.3e+1),cos(x3)./5.3e+1,0.0,0.0,0.0,0.0,0.0,0.0,2.5e+1./4.0],[6,2]);

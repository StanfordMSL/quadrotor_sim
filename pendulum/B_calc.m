function B = B_calc(in1,in2)
%B_calc
%    B = B_calc(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    19-Apr-2022 09:41:48

x01 = in1(1,:);
x02 = in1(2,:);
B = reshape([0.0,x02.*(-2.0./4.5e+1),0.0,sin(x01).*(-4.36e-2)],[2,2]);

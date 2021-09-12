function conx_map = conx_map(in1,in2)
%CONX_MAP
%    CONX_MAP = CONX_MAP(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Sep-2021 03:20:09

map_lim1_1 = in2(1);
map_lim1_2 = in2(4);
map_lim2_1 = in2(2);
map_lim2_2 = in2(5);
map_lim3_1 = in2(3);
map_lim3_2 = in2(6);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
conx_map = [map_lim1_1-x1;-map_lim1_2+x1;map_lim2_1-x2;-map_lim2_2+x2;map_lim3_1-x3;-map_lim3_2+x3];

function [data, info] = attitudeTarget
%AttitudeTarget gives an empty data for mavros_msgs/AttitudeTarget
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.header, info.header] = ros.internal.ros.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.type_mask, info.type_mask] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.IGNORE_ROLL_RATE, info.IGNORE_ROLL_RATE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.IGNORE_PITCH_RATE, info.IGNORE_PITCH_RATE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.IGNORE_YAW_RATE, info.IGNORE_YAW_RATE] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.IGNORE_THRUST, info.IGNORE_THRUST] = ros.internal.ros.messages.ros.default_type('uint8',1, 64);
[data.IGNORE_ATTITUDE, info.IGNORE_ATTITUDE] = ros.internal.ros.messages.ros.default_type('uint8',1, 128);
[data.orientation, info.orientation] = ros.internal.ros.messages.geometry_msgs.quaternion;
info.orientation.MLdataType = 'struct';
[data.body_rate, info.body_rate] = ros.internal.ros.messages.geometry_msgs.vector3;
info.body_rate.MLdataType = 'struct';
[data.thrust, info.thrust] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/AttitudeTarget';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,22);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'type_mask';
info.MatPath{8} = 'IGNORE_ROLL_RATE';
info.MatPath{9} = 'IGNORE_PITCH_RATE';
info.MatPath{10} = 'IGNORE_YAW_RATE';
info.MatPath{11} = 'IGNORE_THRUST';
info.MatPath{12} = 'IGNORE_ATTITUDE';
info.MatPath{13} = 'orientation';
info.MatPath{14} = 'orientation.x';
info.MatPath{15} = 'orientation.y';
info.MatPath{16} = 'orientation.z';
info.MatPath{17} = 'orientation.w';
info.MatPath{18} = 'body_rate';
info.MatPath{19} = 'body_rate.x';
info.MatPath{20} = 'body_rate.y';
info.MatPath{21} = 'body_rate.z';
info.MatPath{22} = 'thrust';

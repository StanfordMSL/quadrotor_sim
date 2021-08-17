function [data, info] = commandBoolResponse
%CommandBool gives an empty data for mavros_msgs/CommandBoolResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'mavros_msgs/CommandBoolResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'mavros_msgs/CommandBoolResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'result';

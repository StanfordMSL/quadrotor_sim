function [data, info] = trajTransferRequest
%TrajTransfer gives an empty data for bridge_px4/TrajTransferRequest
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.hz, info.hz] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.N, info.N] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.u_arr, info.u_arr] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.L_arr, info.L_arr] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'bridge_px4/TrajTransferRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'hz';
info.MatPath{2} = 'N';
info.MatPath{3} = 'u_arr';
info.MatPath{4} = 'L_arr';

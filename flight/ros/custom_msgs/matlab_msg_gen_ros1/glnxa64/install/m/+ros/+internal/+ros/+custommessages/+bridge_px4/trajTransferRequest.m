function [data, info] = trajTransferRequest
%TrajTransfer gives an empty data for bridge_px4/TrajTransferRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'bridge_px4/TrajTransferRequest';
[data.Hz, info.Hz] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.N, info.N] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.UArr, info.UArr] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.XArr, info.XArr] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.LArr, info.LArr] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'bridge_px4/TrajTransferRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'hz';
info.MatPath{2} = 'N';
info.MatPath{3} = 'u_arr';
info.MatPath{4} = 'x_arr';
info.MatPath{5} = 'L_arr';

function [data, info] = trajTransferResponse
%TrajTransfer gives an empty data for bridge_px4/TrajTransferResponse
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.checksum, info.checksum] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'bridge_px4/TrajTransferResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'checksum';

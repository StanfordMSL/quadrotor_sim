clear

bag_add = '/home/lowjunen/Dropbox/Stanford Quarters/[2021] Summer Quarter/ICRA2022/';
testID = 'TEST0';

fullID = [bag_add testID '/'];
full_add = [fullID '*.bag'];
bags = dir(full_add);

for k = 1:length(bags)
    file = [fullID bags(k).name];
    bag = rosbag(file);
    
%     bSel = select(bag,'Topic','/drone2/mavros/vision_pose/pose');
    bSel = select(bag,'Topic','/vrpn_client_node/slit/pose');

    msgStructs = readMessages(bSel,'DataFormat','struct');
end

% bag = rosbag(name);
% bSel = select(bag,'Topic','/drone2/mavros/vision_pose/pose');
% msgStructs = readMessages(bSel,'DataFormat','struct');
% 
% N = size(msgStructs,1);
% 
% for k = 1:N
%     t
% end
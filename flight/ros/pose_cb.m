function pose = pose_cb(msg)
    pose = zeros(7,1);
    
    pose(1,1) = msg.Position.X;
    pose(2,1) = msg.Position.Y;
    pose(3,1) = msg.Position.Z;
    pose(4,1) = msg.Orientation.W;
    pose(5,1) = msg.Orientation.X;
    pose(6,1) = msg.Orientation.Y;
    pose(7,1) = msg.Orientation.Z;
end


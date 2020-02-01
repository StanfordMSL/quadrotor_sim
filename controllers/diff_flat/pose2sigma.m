function sigma = pose2sigma(wp,angle_axis,n_p)

switch angle_axis
    case 'yaw'
        ax_t = 1;
    case 'pitch'
        ax_t = 2;
    case 'roll'
        ax_t = 3;
end

sigma = zeros(4,n_p,wp.N_wp);
for k = 1:wp.N_wp
    index = 11+(3-ax_t);

    quat = wp.x(7:10,k);
    angles = quat2eul(quat');
    sigma(:,:,k) = [wp.x(1:3,k)   wp.x(4:6,k)   zeros(3,n_p-2);...
                    angles(ax_t)  wp.x(index,k) zeros(1,n_p-2)];
                       
end
end

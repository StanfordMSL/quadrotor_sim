function sigma = wp2sigma(wp,angle_axis,n_p)

switch angle_axis
    case 'yaw'
        ax_t = 1;
    case 'pitch'
        ax_t = 2;
    case 'roll'
        ax_t = 3;
end
kf_k = wp.N_wp+1;
sigma.kf = zeros(4,n_p,kf_k);
sigma.t  = zeros(1,kf_k);
for k = 1:wp.N_wp+1
    index = 11+(3-ax_t);

    quat = wp.x(7:10,k);
    angles = quat2eul(quat');
    sigma.kf(:,:,k) = [wp.x(1:3,k)   wp.x(4:6,k)   zeros(3,n_p-2);...
                       angles(ax_t)  wp.x(index,k) zeros(1,n_p-2)];
                       
    sigma.t(1,k) = wp.t(k);
end

sigma.kf_k = kf_k;
end

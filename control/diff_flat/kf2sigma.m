function sigma = kf2sigma(x_kf)
    sigma = zeros(4,15);
    
    sigma(1:3,1) = x_kf(1:3,1);
    sigma(1:3,2) = x_kf(4:6,1);

    q = x_kf(7:10,1)';
    eul = quat2eul(q);
    sigma(4,1) = eul(1);

    q_c = quatconj(q)';
    omega_W = quatrot2(x_kf(7:10,1),q_c);
    sigma(4,2) = omega_W(3);
end
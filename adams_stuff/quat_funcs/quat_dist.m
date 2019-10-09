function ang_dist = quat_dist(q1, q2)
%     % computes the angle between from q1 to q2 in DEGREES
    if(q1(1) < 0)
        q1 = q1 * -1;
    end
    if(q2(1) < 0)
        q2 = q2 * -1;
    end
    q_diff = quatmultiply(q2(:)', quatinv(q1(:)'));
    ang_dist = 2 * acosd(q_diff(1));
    
    disp('');
end
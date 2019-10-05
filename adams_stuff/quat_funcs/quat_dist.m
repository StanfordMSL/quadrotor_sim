function ang_dist = quat_dist(q1, q2)
%     % computes the angle between from q1 to q2 in DEGREES 
%     a = dot(q1(:), q2(:))^2 ;
%     if a < 0
%         a = 0;
%     elseif a > 1
%         a = 1;
%     end
%     ang_dist = abs(acosd(2 * a - 1));
%     
% %   NOTE: ALL THIS IS EQUIVALENT TO:  
%     R1 = quat2rotm(q1(:)');
%     R2 = quat2rotm(q2(:)');
%     axang = rotationMatrixToVector(R2*R1');
%     err = norm(axang) * 180/pi;
%     
%     if(abs(err - ang_dist) > 0.5)
%         warning('ang issue')
%     end
%     
% %   NOTE: ALL THIS IS EQUIVALENT TO:  
%     q_diff = quatmultiply(q2(:)', quatinv(q1(:)'));
%     ang_diff = 2 * acosd(q_diff(1));
%     
%     if(abs(ang_diff - ang_dist) > 0.5)
%         warning('ang issue')
%     end
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
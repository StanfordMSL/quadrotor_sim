function err = HO_calc(traj)

    N = size(traj.x,2);
    err = zeros(13,N);
     
    for k = 2:N-1
        tic
        q_w = traj.x(7,k);
        q_x = traj.x(8,k);
        q_y = traj.x(9,k);
        q_z = traj.x(10,k);
        v_x = traj.x(4,k);
        v_y = traj.x(5,k);
        v_z = traj.x(6,k);

        u1 = traj.u(1,k);
        u2 = traj.u(2,k);
        u3 = traj.u(3,k);
        u4 = traj.u(4,k);
        
        H_x  = H_x_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z);
        H_u  = H_u_calc(q_w,q_x,q_y,q_z);
        H_xu = H_xu_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4);
        
        del_x = traj.x(:,k)-traj.x(:,k-1); 
        del_u = traj.u(:,k)-traj.u(:,k-1); 
            
        for j = 1:13    
            err(j,k) = 0.5.*(del_x'*H_x(:,:,j)*del_x + 2*del_x'*H_xu(:,:,j)*del_u + del_u'*H_u(:,:,j)*del_u);
        end
        toc
    end
    
    figure(5)
    
%     subplot(4,1,1)
%     plot(err(7,:));
%     
%     subplot(4,1,2)
%     plot(err(8,:));
%     
%     subplot(4,1,3)
%     plot(err(9,:));
%     
%     subplot(4,1,4)
%     plot(err(10,:));

    subplot(3,1,1)
    plot(err(11,:));
    
    subplot(3,1,2)
    plot(err(12,:));
    
    subplot(3,1,3)
    plot(err(13,:));
    
end
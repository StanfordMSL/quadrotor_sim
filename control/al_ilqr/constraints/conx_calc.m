function [c, cx, cu] = conx_calc(X,pose_gt,gt_dim,n_x,n_u,N)

% Initialize Variables
c  = zeros(32,N);
cx = zeros(32,n_x,N);
cu = zeros(32,n_u,N);

if (size(pose_gt,2) > 0)
    N_pts = size(gt_dim,2)-1;
    p0  = pose_gt(1:3,1);
    q_gt = quatconj(pose_gt(4:7,1)');
    pts = pose_gt(1:3,1)+quatrotate(q_gt,gt_dim')';

    for k_fr = 1:N
        x_k = X(:,k_fr);

        dist = norm(x_k(1:3)-p0);
        if dist < 0.3
            for k_pts = 1:N_pts
                idx1 = (k_pts-1)*8+1;
                idx2 = 8*k_pts;
                
                p1 = pts(:,k_pts);
                p2 = pts(:,k_pts+1);
                c(idx1:idx2,k_fr)    = conx_gate(x_k,p0,p1,p2);   
                cx(idx1:idx2,:,k_fr) = conx_gate_x(x_k,p0,p1,p2);
            end
        end
    end
end


end
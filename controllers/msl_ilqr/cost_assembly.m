function cost_param = cost_assembly(wts_db,QR_mode,traj,obj_s,model)

N_x = size(traj.x_bar,1);
N_u = size(traj.u_bar,1);
N_s = size(traj.x_bar,2);

cost_param.Q = zeros(N_x,N_x,N_s);
cost_param.R = zeros(N_u,N_u,(N_s-1));

switch QR_mode
    case 'initial'
        for k = 1:(N_s-1)
            cost_param.Q(:,:,k) = wts_db.Q_zero;
            cost_param.R(:,:,k) = wts_db.R_stnd;
        end
        cost_param.Q(:,:,end) = wts_db.Q_unif;
        
    case 'crucial'
        for k = 1:(N_s-1)
            cost_param.R(:,:,k) = wts_db.R_stnd;
        end
        cost_param.Q(:,:,end) = wts_db.Q_unif;
        
%         for k = 1:(N_s-1)
%             wts.Q(:,:,k) = wts_db.Q_pstn;
%             wts.R(:,:,k) = wts_db.R_stnd;
%         end
%         wts.Q(:,:,end) = wts_db.Q_unif;
end

cost_param.x_star = obj_s.x_star;
cost_param.u_star = zeros(4,1);
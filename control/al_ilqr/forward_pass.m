function [Xfp,Ufp,con,La_c] = forward_pass(Xbar,Ubar,T,l,L,La_p,mult,obj)

% Unpack the relevant variables
N = size(Xbar,2);
n_x = size(Xbar,1);
n_u = size(Ubar,1);

xs = obj.kf.x(:,end);
us = zeros(4,1);

FT_ext = zeros(6,1);  
wt = zeros(13,1);

p_box = obj.gt.p_box;

lamx = mult.lamx;
lamu = mult.lamu;
mux = mult.mux;
muu = mult.muu;

% Prepare Container Variables
Xact = zeros(13,N);
Xact(:,1) = obj.kf.x(:,1);

Xfp = zeros(n_x,N);
Xfp(:,1) = obj.kf.x(1:10,1);
Ufp = zeros(n_u,N-1);

con.c = zeros(24,N);
con.cx = zeros(24,n_x,N);
con.cu = zeros(24,n_u,N);

La_c.x_o = zeros(1,N);
La_c.x_c = zeros(1,N);
La_c.u_o = zeros(1,N);
La_c.u_c = zeros(1,N);
La_c.xup = zeros(2,N);

% % Debug
% nominal_plot(X,gate,10,'top');

% while true

br = br_init(); 
for k = 1:N-1
    % Reset the line-search
    c_alpha = 0;
    
    % Generate Reference Value
    Vp = La_p.u_o(k) + La_p.u_c(k) + La_p.x_o(k+1) + La_p.x_c(k+1);
    
    while true
        % Generate alpha
        alpha = -0.2*c_alpha+1;
%         alpha = 0.00001;
        
        % Roll the Dynamic Forward (to get candidates: u_fp,x_fp,br_fp)
        x_now = Xact(:,k);
        del_x = Xfp(:,k)-Xbar(:,k);
        u_fp = Ubar(:,k) + alpha*l(:,k) + L(:,:,k)*del_x;
%         u_fp = Ubar(:,k);

        [u_wr,br_fp] = br_ctrl(x_now,u_fp,br);
        u_mt = w2m_est(u_wr);

        x_act = quadcopter_est(x_now,u_mt,FT_ext,wt);
        x_fp = x_act(1:10,1);

        % Calculate Constraints
        if k == 1
            up = zeros(4,1);
        else
            up = Ufp(:,k-1);
        end

        [conx,conx_x,conx_u] = conx_calc(x_fp,p_box);
        [conu,conu_x,conu_u] = conu_calc(u_fp,up);

        % Check the Constraint
        mudx = check_con(conx,lamx,mux,0);
        mudu = check_con(conu,lamu,muu,0);

        % Check Improvements
        [La_x_o,La_x_c] = lagr_x(x_fp,Xbar(:,k+1),xs,conx,lamx,mudx,T);
        [La_u_o,La_u_c] = lagr_u(u_fp,Ubar(:,k),us,conu,lamu,mudu,T);

        Vc = La_u_o(:,1) + La_u_c(:,1) + La_x_o + La_x_c;
        
        if ( (Vc <= Vp + 1e-4) && (alpha > 0)  )
            % Constraint Cost Improved. Allow a Trajectory Update
            Xact(:,k+1) = x_act;
            Xfp(:,k+1) = x_fp;
            Ufp(:,k) = u_fp;
            br = br_fp;
            
            La_c.x_o(:,k+1) = La_x_o;
            La_c.x_c(:,k+1) = La_x_c;
            La_c.u_o(:,k) = La_u_o(:,1);
            La_c.u_c(:,k) = La_u_c(:,1);
            La_c.xup(:,k) = lagr_xup(Xfp(:,k),Ufp(:,k),Xbar(:,k),Ubar(:,k),xs,us,T);

            con.c(1:16,k+1)  = conx;
            con.cx(1:16,:,k+1) = conx_x;
            con.cu(1:16,:,k+1) = conx_u;
            
            con.c(17:24,k)  = conu(:,1);
            con.cx(17:24,:,k) = conu_x(:,:,1);
            con.cu(17:24,:,k) = conu_u(:,:,1);
            
            break;
        elseif ( (Vc > Vp) && (alpha > 0)  )
            c_alpha = c_alpha+1;
        else
            % Reducing alpha not helping. Resorting to alternatives.
            Xact(:,k+1) = x_act;
            Xfp(:,k+1) = x_fp;
            Ufp(:,k) = u_fp;
            br = br_fp;
            
            La_c.x_o(:,k+1) = La_x_o;
            La_c.x_c(:,k+1) = La_x_c;
            La_c.u_o(:,k) = La_u_o(:,1);
            La_c.u_c(:,k) = La_u_c(:,1);
            La_c.xup(:,k) = lagr_xup(Xfp(:,k),Ufp(:,k),Xbar(:,k),Ubar(:,k),xs,us,T);

            con.c(1:16,k+1)  = conx;
            con.cx(1:16,:,k+1) = conx_x;
            con.cu(1:16,:,k+1) = conx_u;
            
            con.c(17:24,k)  = conu(:,1);
            con.cx(17:24,:,k) = conu_x(:,:,1);
            con.cu(17:24,:,k) = conu_u(:,:,1);
            
            break
        end
    end
end
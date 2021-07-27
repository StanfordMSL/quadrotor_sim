function [Xfp,Ufp,con,La_c,mult] = forward_pass(Xbar,Ubar,T,l,L,La_p,mult,obj)

% Unpack the relevant variables
N = size(Xbar,2);
n_x = size(Xbar,1);
n_u = size(Ubar,1);

xs = obj.kf.x(:,end);
us = round(Ubar(:,end),3);

FT_ext = zeros(6,1);  
wt = zeros(13,1);

p_box = obj.gt.p_box;

lamx = mult.lamx;
lamu = mult.lamu;
mux = mult.mux;
muu = mult.muu;

Xfp_s = Xbar;
Ufp_s = Ubar;
con_s = con_calc(Xfp_s,Ufp_s,p_box);
La_s  = La_p;
mult_s = mult;

% % Debug
% nominal_plot(X,gate,10,'top');

alpha = 1;
while true  
    % Initialize body-rate PID
    br = br_init();
    
    % Prepare Container Variables
    Xact = zeros(13,N);
    Xact(:,1) = obj.kf.x(:,1);

    Xfp = zeros(n_x,N);
    Xfp(:,1) = obj.kf.x(1:10,1);
    Ufp = zeros(n_u,N-1);
       
    % Roll the Dynamic Forward (to get candidates: u_fp,x_fp)
    for k = 1:N-1
        x_now = Xact(:,k);
        del_x = Xfp(:,k)-Xbar(:,k);
        u_fp = Ubar(:,k) + alpha*l(:,k) + L(:,:,k)*del_x;

        [u_wr,br] = br_ctrl(x_now,u_fp,br);
        u_mt = w2m_est(u_wr);

        Xact(:,k+1) = quadcopter_est(x_now,u_mt,FT_ext,wt);
        Xfp(:,k+1)  = Xact(1:10,k+1);
        Ufp(:,k)    = u_fp;
    end
    
    % Calculate Constraints
    con = con_calc(Xfp,Ufp,p_box);
    conx = con.c(1:16,:);
    conu = con.c(17:end,:);
    
    % Calculate the Lagrangian
    mult.mudx = check_con(conx,lamx,mux,0);
    mult.mudu = check_con(conu,lamu,muu,0);
    
    [La_c.x_o,La_c.x_c] = lagr_x(Xfp,Xbar,xs,conx,lamx,mult.mudx,T);
    [La_c.u_o,La_c.u_c] = lagr_u(Ufp,Ubar,us,conu,lamu,mult.mudu,T);
    La_c.xup = lagr_xup(Xfp,Ufp,Xbar,Ubar,xs,us,T);

    La_c.obj = sum(La_c.x_o) + sum(La_c.u_o) + sum(sum(La_c.xup));
    La_c.con = sum(La_c.x_c) + sum(La_c.u_c);
    La_c.tot = La_c.obj + La_c.con;

    if (La_c.con <= (La_p.con + 1e-3) && (alpha > 0.001))
        break;
    elseif (La_c.con > (La_p.con + 1e-3) && (alpha > 0.001))
        alpha = 0.5*alpha;
    else
        % Line-Search will not produce any improvements. Resort to other
        % methods of improvement (Lagrange multipliers).
        Xfp  = Xfp_s;
        Ufp  = Ufp_s;
        con  = con_s;
        La_c = La_s;
        mult = mult_s;
        
        break;
    end
end

% La_plot(La_p,La_c)

end
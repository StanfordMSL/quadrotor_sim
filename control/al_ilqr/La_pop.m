function [X,U,con,La,alpha] = La_pop(Xbar,Ubar,lqr,l,L,pose_gt,gt_dim,map,mult)

La_arr = zeros(3,10);
alpha_0 = 0.8;
La_arr(1,:) = [alpha_0.^(0:1:8) 0];

for k = 1:10
    [X,U] = forward_pass(Xbar,Ubar,l,L,La_arr(1,k));
    con  = con_calc(X,U,pose_gt,gt_dim,map);
    mult = mult_check(con,mult,0);
    La = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult);

    La_arr(2,k) = La.con;
    La_arr(3,k) = La.obj;
    
    
    if k > 1
        if La_arr(2,k) > La_arr(2,k-1)
            break;
        end
    end
end

[~,idx] = min(La_arr(2,:));
alpha = La_arr(1,idx);

[X,U] = forward_pass(Xbar,Ubar,l,L,alpha);
con  = con_calc(X,U,pose_gt,gt_dim,map);
mult = mult_check(con,mult,0);
La = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult);
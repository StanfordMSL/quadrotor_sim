function [con,con_x,con_u] = motor_con(u_bar,model)

    kt =  model.kt_est(1,1);
    w2m = model.m2w_inv;
    
    f_min = kt.*(model.motor_min.^2);
    f_max = kt.*(model.motor_max.^2);
    
    f_m = model.m2w_inv*u_bar(1:4,1);
    
    con = zeros(8,1);
    con_u = zeros(8,4);
    con_x = zeros(8,13);

%     for k = 1:4
%         con(k,1)   = -f_m(k,1) + f_min;
%         con(k+4,1) =  f_m(k,1) - f_max;
% 
%         con_u(k,:)   = -w2m(k,:);
%         con_u(k+4,:) =  w2m(k,:);
%     end
%     
end
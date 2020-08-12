function f_m = motor_transform(u_curr,model,conv_type)
    
    f_m = model.m2w_inv*u_curr;
%     f_m = lsqnonneg(model.m2w,u_curr);
    
    switch conv_type
        case 'actual'
            kt =  model.kt_act(1,1);
            
            f_min = kt*(model.motor_min^2);
            f_max = kt*(model.motor_max^2);
            
            for k = 1:4
                if f_m(k,1) > f_max
                    f_m(k,1) = f_max;
                elseif f_m(k,1) < f_min
                    f_m(k,1) = f_min;
                end
            end
        case 'sim'
            % passthrough to allow augmented lagrangian to maintain
            % continuous search.
    end
end
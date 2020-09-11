function sigma = obj2sigma(obj,n_p,N_wp)


if norm(obj.p_g) < 999
    % We have a gate. Need to inject it into the array.
%     x_obj = [obj.p_g ; 0 ; 0 ; 1 ; 1 ; zeros(6,1)];
    x_obj = [obj.p_g ; 2 ; 0 ; 0 ; 1 ; zeros(6,1)];
    
    N_obj = N_wp + 1;
    x_wp = [ obj.wp_arr(:,1) x_obj obj.wp_arr(:,end)];
else
    % We don't have a gate. Carry on.
    N_obj = N_wp;
    x_wp = [ obj.wp_arr(:,1) obj.wp_arr(:,end)];
end

sigma = zeros(4,n_p,N_obj);
for k = 1:N_obj
    index = 11+(3-1);

    quat = x_wp(7:10,k);
    angles = quat2eul(quat');
    sigma(:,:,k) = [x_wp(1:3,k)   x_wp(4:6,k)       zeros(3,n_p-2);...
                    angles(1,1)   x_wp(index,k)     zeros(1,n_p-2)];                    
end

end

function sig_points = UT(mu_curr,sigma_curr,dim,lambda,quat_flag)   
    sig_points = zeros(dim,(1+2*dim));
 
    sig_points(:,1) = mu_curr(:,1);
    
    for k = 1:dim
        sqrt_mat = sqrtm(sigma_curr);
%         [U,S,~] = svd(sigma_curr);
%         sqrt_mat = U*sqrtm(S);
        
        matrix = sqrt(dim+lambda)*sqrt_mat;
        sig_points(:,k+1)     = mu_curr(:,1) + matrix(:,k);
        sig_points(:,dim+k+1) = mu_curr(:,1) - matrix(:,k);
    end
    
    if (quat_flag == 1)
        for k = 1:(1+2*dim)
            sig_points(7:10,k) = sig_points(7:10,k)./norm(sig_points(7:10,k));
        end
    end
end
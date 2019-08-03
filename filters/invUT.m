function [mu, sigma] = invUT(sigma_points,dim,lambda,quat_flag)
    w0 = lambda/(lambda+dim);
    wi = 1/(2*(lambda+dim));
    
    mu = (w0 * sigma_points(:,1)) + (wi*sum(sigma_points(:,2:end),2));
    if (quat_flag == 1)
        mu(7:10,1) = mu(7:10,1)./norm(mu(7:10,1));
    end
    
    sigma = w0*(sigma_points(:,1)-mu)*(sigma_points(:,1)-mu)';
    for k = 1:dim
        sigma = sigma + wi*(sigma_points(:,k+1)-mu)*(sigma_points(:,k+1)-mu)';
        sigma = sigma + wi*(sigma_points(:,dim+k+1)-mu)*(sigma_points(:,dim+k+1)-mu)';
    end
end
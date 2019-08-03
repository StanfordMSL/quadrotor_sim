function sigma_xy = cross_cov(mu1,sigmap1,mu2,sigmap2,dim,lambda)
    w0 = lambda/(lambda+dim);
    wi = 1/(2*(lambda+dim));
    
    sigma_xy = w0*(sigmap1(:,1)-mu1)*(sigmap2(:,1)-mu2)';
    for k = 1:dim
        sigma_xy = sigma_xy + wi*(sigmap1(:,k+1)-mu1)*(sigmap2(:,k+1)-mu2)';
        sigma_xy = sigma_xy + wi*(sigmap1(:,dim+k+1)-mu1)*(sigmap2(:,dim+k+1)-mu2)';
    end
end
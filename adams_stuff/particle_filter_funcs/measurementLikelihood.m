function likelihood = measurementLikelihood(pf,predictParticles,measurement,measurement_cov,camera, initial_bb,yukf)
%MEASUREMENTLIKELIHOOD Summary of this function goes here
%   Detailed explanation goes here
%     measurement_cov = eye(5)*5;
%     measurement_cov(5,5) = 0.02;
%     measurement_cov(1,1) = 2;
%     measurement_cov(2,2) = 2;
%   
    pred_obs = zeros(pf.NumParticles,length(measurement));
    for i = 1:pf.NumParticles
        pred_obs(i,:) = predict_obs(predictParticles(i,:)', camera, initial_bb, yukf);
    end
    
    likelihood = mvnpdf(pred_obs,measurement',measurement_cov);
    
    
        
end


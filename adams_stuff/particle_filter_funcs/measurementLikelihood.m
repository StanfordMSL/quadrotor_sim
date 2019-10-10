function likelihood = measurementLikelihood(pf,predictParticles,measurement,measurement_cov,camera, initial_bb,yukf)
%MEASUREMENTLIKELIHOOD Summary of this function goes here
%   Detailed explanation goes here
%     measurement_cov = eye(5)*5;
%     measurement_cov(5,5) = 0.02;
%     measurement_cov(1,1) = 2;
%     measurement_cov(2,2) = 2;
%   
    pred_obs = zeros(pf.NumParticles,length(measurement));
    axang_3_dim = predictParticles(:,7:9);
    ang_val = sqrt(sum(axang_3_dim.^2,2));
    predictParticles_quat = [predictParticles(:,1:6) axang2quat([axang_3_dim./ang_val ang_val]) predictParticles(:,10:12)];
    num = pf.NumParticles;
    parfor i = 1:num
        pred_obs(i,:) = predict_obs(predictParticles_quat(i,:)', camera, initial_bb, yukf);
    end
    
    likelihood = mvnpdf(pred_obs,measurement',measurement_cov);
    
    
        
end


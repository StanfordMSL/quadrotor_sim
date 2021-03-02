function y = sensors(x_act,model)
    % Unpack some stuff
    C = model.C;
    R = model.R_fil;
    
    % Generate Noise
    vt = R*randn(10,1);
    
    y = C*x_act + vt;
end
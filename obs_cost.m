function [dc_x, dc_xx] = obs_cost(x,b,c,targ)
    px = x(1,1);
    py = x(2,1);
    pz = x(3,1);
    
    ax = targ(1,1);
    ay = targ(2,1);
    az = targ(3,1);
    
    % Power 2
    dx = (c*(2*ax - 2*px))/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^2);
    dy = (c*(2*ay - 2*py))/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^2);
    dz = (c*(2*az - 2*pz))/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^2);
    
    dxx = (2*c*(2*ax - 2*px)^2)/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^3) - (2*c)/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^2);
    dyy = (2*c*(2*ay - 2*py)^2)/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^3) - (2*c)/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^2);
    dzz = (2*c*(2*az - 2*pz)^2)/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^3) - (2*c)/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^2);
    
    dxy = (2*c*(2*ax - 2*px)*(2*ay - 2*py))/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^3);
    dxz = (2*c*(2*ax - 2*px)*(2*az - 2*pz))/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^3);
    dyz = (2*c*(2*ay - 2*py)*(2*az - 2*pz))/(b^2*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^3);
    
%     % Power 3
%     dx = (3*c*(2*ax - 2*px))/(2*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(5/2));
%     dy = (3*c*(2*ay - 2*py))/(2*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(5/2));
%     dz = (3*c*(2*az - 2*pz))/(2*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(5/2));
% 
%     dxx = (15*c*(2*ax - 2*px)^2)/(4*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(7/2)) - (3*c)/(b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(5/2));
%     dyy = (15*c*(2*ay - 2*py)^2)/(4*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(7/2)) - (3*c)/(b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(5/2));
%     dzz = (15*c*(2*az - 2*pz)^2)/(4*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(7/2)) - (3*c)/(b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(5/2));
%     
%     dxy = (15*c*(2*ax - 2*px)*(2*ay - 2*py))/(4*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(7/2));
%     dxz = (15*c*(2*ax - 2*px)*(2*az - 2*pz))/(4*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(7/2));
%     dyz = (15*c*(2*ay - 2*py)*(2*az - 2*pz))/(4*b^3*((ax - px)^2 + (ay - py)^2 + (az - pz)^2)^(7/2));

% Packaging
    dc_x = [dx ; dy ; dz ; zeros(10,1)];
    
    dc_xx = zeros(13,13);
    dc_xx(1:3,1:3) = [dxx dxy dxz ; 
                      dxy dyy dyz ;
                      dxz dxy dzz ];
end
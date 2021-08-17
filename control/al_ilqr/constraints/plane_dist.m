function d = plane_dist(c,p)

% c is the plane equation (Ax + By +Cz + D = 0) in 4x1 form
% p is the point

num = abs(c(1)*p(1)+c(2)*p(2)+c(3)*p(3)+c(4));
den = norm(c(1:3));

d = num/den;
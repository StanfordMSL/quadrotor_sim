function plane_dist_init()

x = sym('x',[3 1],'real');
p = sym('p',[3 3],'real');

p12 = p(:,2)-p(:,1);
p13 = p(:,3)-p(:,1);

n = cross(p12,p13);
c = [ n ; -n'*p(:,1)];

num = abs(c(1)*x(1)+c(2)*x(2)+c(3)*x(3)+c(4));
den = norm(c(1:3));

d = num/den;

address = 'control/al_ilqr/constraints/';
matlabFunction(d ,'File',[address,'plane_dist_calc'],'vars',{x,p});

end
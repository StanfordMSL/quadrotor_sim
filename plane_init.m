clear

x = sym('x',[3 1],'real');
p0 = sym('p0',[3 1],'real');
p1 = sym('p1',[3 1],'real');
p2 = sym('p2',[3 1],'real');

p12 = p2-p1;
p10 = p0-p1;

t  = (p10'*p12)/(p12'*p12);
pn = p1 + t*(p2-p1);
n  = p0-pn;

plane = n(1)*(x(1)-pn(1))+n(2)*(x(2)-pn(2))+ n(3)*(x(3)-pn(3));
matlabFunction(plane ,'File','plane_calc','vars',{x,p0,p1,p2});


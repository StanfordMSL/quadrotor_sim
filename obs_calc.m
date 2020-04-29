clear 

syms f(px,py,pz,ax,ay,az,b,c)

f(px,py,pz,ax,ay,az,b,c) = c*(b*((px-ax)^2+(py-ay)^2+(pz-az)^2)^(0.5))^(-2);

dx = diff(f,px)
dy = diff(f,py)
dz = diff(f,pz)

dxx = diff(dx,px)
dyy = diff(dy,py)
dzz = diff(dz,pz)

dxy = diff(dx,py)
dxz = diff(dx,pz)
dyz = diff(dy,pz)

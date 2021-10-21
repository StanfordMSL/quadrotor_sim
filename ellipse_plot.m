function ellipse_plot(x0,A)

[V,D] = eig(A);

[theta,phi] = ndgrid(linspace(0,pi),linspace(0,2*pi));

r = 0.001;

a = sqrt(r/D(1,1));
b = sqrt(r/D(2,2));
c = sqrt(r/D(3,3));

X = a.*sin(theta).*cos(phi);
Y = b.*sin(theta).*sin(phi);
Z = c.*cos(theta);

a1 = zeros(100,100);
a2 = zeros(100,100);
a3 = zeros(100,100);

temp=[X(:),Y(:),Z(:),a1(:),a2(:),a3(:)]*V.';
      
sz=size(X);

Xrot=x0(1)+reshape(temp(:,1),sz);
Yrot=x0(2)+reshape(temp(:,2),sz);
Zrot=x0(3)+reshape(temp(:,3),sz);

surf(Xrot,Yrot,Zrot)

end


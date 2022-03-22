function output = recursive(traj_nom,traj_act)

x  = sym('x',[13 1],'real');
u  = sym('u',[4 1],'real'); 
th = sym('th',[7 1],'real'); 
s = [x;u;th];

f = quad_sim(x,u,th,200);

Dx  = jacobian(f,x);
Dx = simplify(Dx);

Dth = jacobian(f,th);
Dth = simplify(Dth);

c1 = eye(13);

output = zeros(1,20);

function rA = recA(X,)
for k = 1:3
    % Unpack 
    xk = traj_act.X(:,k);
    uk = traj_act.U(:,k);
    xb = traj_nom.X(:,k);
    
    fk = subs(f,u,uk);
    
%     thk = [model.m ; model.Ipp ; model.Df(1:3,1)];
%     sk = [xk ; uk ; thk];

    % Calculate Output
    delx = xk-xb;
    c1p = c1;
    c1 = Dx;
    c3 = Dth;

    if k == 1
        c2 = Dth;
    else
        c2 = C;
    end
    C = c1*c2+c3;

    output = output+delx'*[C c1*c1p];
end
function output = brute_force(traj)

xo = sym('xo',[13 1],'real');
theta  = sym('z',[7 1],'real'); 

N = 3;
J = 0;
x = xo;
for k = 1:N
    % Unpack 
    xk = traj.X(:,k);
    uk = traj.U(:,k);

    % Calculate Cost
    delx = x-xk;
    J = J + 0.5*(delx'*delx);
    J = simplify(J);

    if k < N
        x = quad_sim(x,uk,theta,200);
    end
end

s = [xo ; theta];
eqns = jacobian(J,s) == zeros(20,1);

output = solve(eqns,s);
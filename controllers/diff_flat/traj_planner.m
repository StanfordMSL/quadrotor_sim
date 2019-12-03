function [t_out,f_out] = traj_planner(sigma,con_hz,n_p)
tic
mu_r = 1;
mu_psi = 1;
k_r = 4;
OPTIONS = optimoptions('fmincon','Algorithm','sqp','Display','off');

A_sigma = zeros(4,n_p);
for i = 1:4
    k_row = (2*5) + ((sigma.kf_k-2)*2);
    Aeq = zeros(k_row,n_p);
    Beq = zeros(k_row,1);
    index = 1;
    
    for j = 1:sigma.kf_k
        t_kf = sigma.t(1,j);
        if (j == 1) || (j == sigma.kf_k)
            for k = index:index+k_r
                for m = 1:n_p
                    if ((k+m-index) <= n_p)
                        Aeq(k,(k+m-index)) = (t_kf^(m-1))/factorial(m-1);
                    end
                end
            end
            Beq(index:index+k_r,1) = sigma.kf(i,1:5,j)';
            index = index+k_r+1;
        else
            for m = 1:n_p
                Aeq(index,m) = t_kf^(m-1)/factorial(m-1);
            end
            Beq(index,1) = sigma.kf(i,1,j);
            index = index+1;
        end
    end

    f = @min_func;
    if k == 4
        FUN = @(x)integral(@(t) mu_psi*(f(t,x,n_p))^2,sigma.t(1),sigma.t(end),'ArrayValued',true);
    else
        FUN = @(x)integral(@(t) mu_r*(f(t,x,n_p))^2,sigma.t(1),sigma.t(end),'ArrayValued',true);
    end
        
    A_sigma(i,:) = fmincon(FUN,sigma.kf(i,:,1)',[],[],Aeq,Beq,[],[],[],OPTIONS);
end

count = con_hz*sigma.t(end)+1;
t_out = linspace(0,sigma.t(end),count);
f_out = zeros(4,5,count);

for k = 1:count
    t_kf = t_out(k);
    
    B_sigma = zeros(n_p,n_p);
    for j = 1:n_p
        b_vec = ((t_kf^(j-1))/factorial(j-1))*ones(n_p-j+1,1);
        B_sigma = B_sigma + diag(b_vec,-j+1);
    end
    feed = A_sigma * B_sigma;
    f_out(:,:,k) = feed(:,1:5);
end
toc
end
function J = J_calc(Xhat,Xact)
    J = sum(vecnorm(Xhat-Xact));
end
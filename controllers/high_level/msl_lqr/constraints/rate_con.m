function [con,con_x,con_u] = rate_con(x_bar)
    %% Rate Constraints
%     A = [zeros(3,10) eye(3)];
%     
%     con_low = -A*x_bar - 1.5.*ones(3,1);
%     con_upp =  A*x_bar - 1.5.*ones(3,1);
%     
%     con = [con_low ; con_upp];
%     con_x = [ -A ; A];
%     con_u = zeros(6,4);
    
    con = zeros(6,1);
    con_x = zeros(6,13);
    con_u = zeros(6,4);
end
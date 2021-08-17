function gate_debug(x,p_box)

% Unpack Some Stuff
l_arm   = 0.06;
r_d_arr = [ l_arm    0.00  -l_arm    0.00;
            0.00  -l_arm    0.00   l_arm; 
            0.00    0.00    0.00    0.00 ];
n_p = size(r_d_arr,2);

p_G1 = p_box(:,1);
p_G2 = p_box(:,2);
p_G4 = p_box(:,4);
r_g_arr = [p_G2-p_G1 p_G4-p_G1];

gain = zeros(8,1);
r_d = zeros(3,4);

% Clean Up Fig Space
figure(1)
clf

% Plot Gate Box
gate_box = [p_box(2:3,:) p_box(2:3,1)];

plot(gate_box(1,:),gate_box(2,:));
hold on

% Four Corner Compute
for m = 1:size(x,2)
    for j = 1:n_p

        r_d(:,j) = x(1:3,m) + quatrot2(r_d_arr(:,j),x(7:10,m));

        for k = 1:2
            r_g = r_g_arr(:,k);
        
            idx = (j-1)*2+k;

            gain(idx,1) = dot((r_d(:,j)-p_G1),r_g)./(r_g'*r_g);
        end
    end
    
    l1 = [r_d(2:3,1) r_d(2:3,3)];
    l2 = [r_d(2:3,2) r_d(2:3,4)];
    
    plot(l1(1,:),l1(2,:));
    plot(l2(1,:),l2(2,:));
end



% for j = 1:2
%     r_g = r_g_arr(:,j);
% 
%     for k = 1:n_p
%         idx = (j-1)*n_p+k;
% 
%         r_d = x(1:3,1) + quatrot2(r_d_arr(:,k),x(7:10,1));
%         gain(idx,1) = dot((r_d-p_G1),r_g)./(r_g'*r_g);
%     end
% end
% 
% conx   = [-gain ; gain-ones(n_p*2,1)];



hold on
plot(r_d(2,:),r_d(3,:),'*');

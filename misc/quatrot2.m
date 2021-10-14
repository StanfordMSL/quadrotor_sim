function vect_out = quatrot2(vect_in,quat_in)

vect = vect_in;
quat = quat_in;
quat_c = [quat_in(1) ; -quat_in(2) ; -quat_in(3); -quat_in(4)];

index_array = [ 1  2  3  4 ;
                2 -1  4 -3 ;
                3 -4 -1  2 ;
                4  3 -2 -1];
           
vect_intr = sym(zeros(4,1));
vect_out  = sym(zeros(4,1));

vect_feed = [0 ; vect];
for j = 1:4
    for k = 1:4
        data = index_array(j,k);
        index = abs(data);
        coeff = sign(data);
        
        vect_intr(index,1) = vect_intr(index,1) + coeff.*quat(j,1)*vect_feed(k,1);
    end
end

for j = 1:4
    for k = 1:4
        data = index_array(j,k);
        index = abs(data);
        coeff = sign(data);
        
        vect_out(index,1) = vect_out(index,1) + coeff.*vect_intr(j,1)*quat_c(k,1);
    end
end
vect_out = simplify(vect_out(2:4,1));

             
function mat_out = quatrotmat2(mat_in,quat_in)

mat_out = sym(zeros(3,3));
for k=1:3
    vect_in = mat_in(:,k);
    mat_out(:,k) = quatrot2(vect_in,quat_in);
end
function tf_inv = inv_tf(tf)
    tf_inv = eye(4);
    tf_inv(1:3, 1:3) = tf(1:3, 1:3)';
    tf_inv(1:3, 4) = -tf_inv(1:3, 1:3) * tf(1:3, 4); 
end
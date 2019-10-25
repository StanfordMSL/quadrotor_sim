function rc = xyz_to_rc(xyz, camera)
    % input: assumes xyz are in camera frame
    % output: [row, col] i.e. the projection of xyz onto camera plane
    rc = camera.K * reshape(xyz(1:3), 3, 1);
    rc = [rc(2), rc(1)] / rc(3);
end
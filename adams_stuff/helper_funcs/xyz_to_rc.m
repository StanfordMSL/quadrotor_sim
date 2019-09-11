function rc = xyz_to_rc(xyz, camera)
    % output: [row, col] i.e. the projection of xyz onto camera plane
    rc = camera.K * xyz(:);
    rc = [rc(2), rc(1)] / rc(3);
end
function rc = xyz_to_rc(xyz, camera)
    rc = camera.K_3x3 * xyz(:);
    rc = [rc(2), rc(1)] / rc(3);
end
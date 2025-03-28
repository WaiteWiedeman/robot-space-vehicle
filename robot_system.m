function dxdt = robot_system(t, x, sysParams, ctrlParams)
    [Xd, Yd, Xdd, Ydd, Xv, Xvd, Yv, Yvd] = referenceTrajectory(t, ctrlParams,sysParams);
    [Alv,Th1,Th2,Alvd,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xv,Xvd,Yv,Yvd);
    if ctrlParams.noise
        xm = addnoise(t, x, ctrlParams);
        F = force_function(t, xm, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
    else
        F = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
    end
    dxdt = robot_xdot(x, F, sysParams);
end
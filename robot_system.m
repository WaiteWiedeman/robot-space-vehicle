function dxdt = robot_system(t, x, sysParams, ctrlParams)
    [Xd, Yd, Xdd, Ydd, Xv, Xvd, Yv, Yvd] = referenceTrajectory(t, ctrlParams,sysParams);
    [Alv,Th1,Th2,Alvd,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xv,Xvd,Yv,Yvd);
    % disp(size(x))
    if ctrlParams.noise
        xm = addnoise(x, ctrlParams);
        % disp(size(xm))
        if exist("noise.mat", 'file') == 2
            xms = load("noise.mat", 'xms').xms;
            % disp(xms)
            xms(:,end+1) = xm;
            % disp(xms)
            % disp(xm)
        else
            xms = zeros(size(x));
            % disp(xm)
            xms(:,end) = xm;
            % disp(xms(:,end))
            % disp(xm)
        end
        save("noise.mat", 'xms');
        % disp("robot_system")
        % disp(xm)
        % disp(x)
        % disp(xm-x)
        F = force_function(t, xm, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
    else
        F = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
    end
    dxdt = robot_xdot(x, F, sysParams);
end
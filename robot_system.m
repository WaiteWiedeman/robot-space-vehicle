function dxdt = robot_system(t, x, noise, sysParams, ctrlParams)
    % persistent count Fp tp
    % if isempty(count) || isempty(Fp) || isempty(tp) || t == 0
    %     count = 1;
    %     Fp = zeros(5,1);
    %     tp = [];
    % end
    [Xd, Yd, Xdd, Ydd, Xv, Xvd, Yv, Yvd] = referenceTrajectory(t, ctrlParams,sysParams);
    [Alv,Th1,Th2,Alvd,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xv,Xvd,Yv,Yvd);
    % if ctrlParams.noise
    %     % xm = addnoise(t, x, ctrlParams);
    %     Fg = force_function(t, xm, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
    %     F = Fp + ctrlParams.Pf*(Fg-Fp);
    %     disp(F)
    %     disp(count)
    % else
    %     Fg = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
    %     F = Fp + ctrlParams.Pf*(Fg-Fp);
    %     disp(F)
    %     disp(count)
    % end
    F = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, noise, ctrlParams);
    dxdt = robot_xdot(x, F, sysParams);
end
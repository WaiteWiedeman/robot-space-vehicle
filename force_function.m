function F = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams)
    persistent ti e1 e2 e3 e4 e5
    if t == 0
        ti = [];
        e1 = [];
        e2 = [];
        e3 = [];
        e4 = [];
        e5 = [];
    end
    ti(end+1) = t;

    F = zeros(5,1);

    xv = x(1); % vehicle x position
    xvd = x(2); % vehicle x velocity
    yv = x(3); % vehicle y position
    yvd = x(4); % vehicle y velocity
    alv = x(5); % vehicle pitch angle
    alvd = x(6); % vehicle pitch velocity
    th1 = x(7); % link 1 position
    th1d = x(8); % link 1 velocity
    th2 = x(9); % link 2 position
    th2d = x(10); % link 2 velocity

    PID1 = ctrlParams.PID1;
    PID2 = ctrlParams.PID2;
    PID3 = ctrlParams.PID3;
    PID4 = ctrlParams.PID4;
    PID5 = ctrlParams.PID5;

    e1(end+1) = Xv - xv;
    e1d = Xvd - xvd;
    F(1) = PID1(1)*e1(end) + PID1(2)*trapz(ti,e1,2) + PID1(3)*e1d;
    if F(1) > 200
        F(1) = 200;
    elseif F(1) < -200
        F(1) = -200;
    end

    e2(end+1) = Yv - yv;
    e2d = Yvd - yvd;
    F(2) = PID2(1)*e2(end) + PID2(2)*trapz(ti,e2,2) + PID2(3)*e2d;
    if F(2) > 200
        F(2) = 200;
    elseif F(2) < -200
        F(2) = -200;
    end

    e3(end+1) = Alv - alv;
    e3d = Alvd - alvd;
    F(3) = PID3(1)*e3(end) + PID3(2)*trapz(ti,e3,2) + PID3(3)*e3d;
    if F(3) > 200
        F(3) = 200;
    elseif F(3) < -200
        F(3) = -200;
    end

    e4(end+1) = Th1 - th1;
    e4d = Om1 - th1d;
    F(4) = PID4(1)*e4(end) + PID4(2)*trapz(ti,e4,2) + PID4(3)*e4d;
    if F(4) > 200
        F(4) = 200;
    elseif F(4) < -200
        F(4) = -200;
    end

    e5(end+1) = Th2 - th2;
    e5d = Om2 - th2d;
    F(5) = PID5(1)*e5(end) + PID5(2)*trapz(ti,e5,2) + PID5(3)*e5d;
    if F(5) > 200
        F(5) = 200;
    elseif F(5) < -200
        F(5) = -200;
    end
end

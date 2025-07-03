function F = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, noise, ctrlParams)
    persistent ti e1 e2 e3 e4 e5 Fp count
    % disp(t)
    if t == 0 || isempty(ti)
        ti = t;
        Fp = zeros(5,1);
        e1 = [];
        e2 = [];
        e3 = [];
        e4 = [];
        e5 = [];
        count = 1;
    else
        ti(end+1) = t;
    end 

    if count == length(noise)
        count = 0;
    end
    % if length(ti)>2
    %     if ti(end)-ti(end-1) > 0.001
    %         count = count + 1;
    %     end
    % end
    count = count + 1;
    % disp(count)
    % disp(size(ti))
    Fg = zeros(5,1);
    Flim = ctrlParams.Flim;
    Tlim = ctrlParams.Tlim;
    % disp("force_function")
   
    if ctrlParams.noise
        x = x.*(ones(size(x)) + noise(:,count));
    end
    
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
    Fg(1) = PID1(1)*e1(end) + PID1(2)*trapz(ti,e1,2) + PID1(3)*e1d;
    if Fg(1) > Flim
        Fg(1) = Flim;
    elseif Fg(1) < -Flim
        Fg(1) = -Flim;
    end

    e2(end+1) = Yv - yv;
    e2d = Yvd - yvd;
    Fg(2) = PID2(1)*e2(end) + PID2(2)*trapz(ti,e2,2) + PID2(3)*e2d;
    if Fg(2) > Flim
        Fg(2) = Flim;
    elseif Fg(2) < -Flim
        Fg(2) = -Flim;
    end

    e3(end+1) = Alv - alv;
    e3d = Alvd - alvd;
    Fg(3) = PID3(1)*e3(end) + PID3(2)*trapz(ti,e3,2) + PID3(3)*e3d;
    if Fg(3) > Tlim
        Fg(3) = Tlim;
    elseif Fg(3) < -Tlim
        Fg(3) = -Tlim;
    end

    e4(end+1) = Th1 - th1;
    e4d = Om1 - th1d;
    Fg(4) = PID4(1)*e4(end) + PID4(2)*trapz(ti,e4,2) + PID4(3)*e4d;
    if Fg(4) > Tlim
        Fg(4) = Tlim;
    elseif Fg(4) < -Tlim
        Fg(4) = -Tlim;
    end

    e5(end+1) = Th2 - th2;
    e5d = Om2 - th2d;
    Fg(5) = PID5(1)*e5(end) + PID5(2)*trapz(ti,e5,2) + PID5(3)*e5d;
    if Fg(5) > Tlim
        Fg(5) = Tlim;
    elseif Fg(5) < -Tlim
        Fg(5) = -Tlim;
    end
    % disp(F)
    % u_max = ctrlParams.Pf; % *ones(5,1);
    % F(1) = Fp(1) + clip(Fg(1)-Fp(1),-u_max,u_max); 
    % F(2) = Fp(2) + clip(Fg(2)-Fp(2),-u_max,u_max);
    % F(3) = Fp(3) + clip(Fg(3)-Fp(3),-u_max,u_max);
    % F(4) = Fp(4) + clip(Fg(4)-Fp(4),-u_max,u_max);
    % F(5) = Fp(5) + clip(Fg(5)-Fp(5),-u_max,u_max);
    F = Fp + ctrlParams.Pf*(Fg-Fp);
    Fp = F;
end

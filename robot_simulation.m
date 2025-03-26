function y = robot_simulation(tSpan, x0, sysParams, ctrlParams)
    % ODE solver
    if ctrlParams.fixedTimeStep ~= 0
        tSpan = tSpan(1):ctrlParams.fixedTimeStep:tSpan(2);
    end

    switch ctrlParams.solver
        case "nonstiff"
            [t,x] = ode45(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0);
        case "stiffhr"
            opts = odeset('RelTol',1e-7,'AbsTol',1e-9); 
            [t,x] = ode15s(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0,opts); 
        case "stifflr"
            opts = odeset('RelTol',1e-4,'AbsTol',1e-5); 
            [t,x] = ode15s(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0,opts);
        case "GA"
            startTime = datetime;
            stopTime = 60; % end sim in 60 seconds
            opts = odeset('RelTol',1e-4,'AbsTol',1e-5,'OutputFcn', @(t, y, flag) myOutputFcn(t, y, flag, startTime, stopTime));
            [t,x] = ode15s(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0, opts);
    end
    [t,x] = select_samples(ctrlParams, t, x);
    numTime = length(t);
    if ctrlParams.noise
        xms = load("noise.mat", 'xms').xms;
        y = zeros(numTime, 41);
    else
        y = zeros(numTime, 31); 
    end
    for i = 1 : numTime
        [Xd, Yd, Xdd, Ydd, Xv, Xvd, Yv, Yvd] = referenceTrajectory(t(i), ctrlParams,sysParams);
        [Alv,Th1,Th2,Alvd,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xv,Xvd,Yv,Yvd);
        if ctrlParams.noise
            xm = xms(:,i);
            F = force_function(t(i), xm', Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
        else
            F = force_function(t(i), x(i,:), Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, ctrlParams);
        end
        xdot = robot_xdot(x(i,:), F, sysParams);
        y(i,1) = t(i); % t
        y(i,2) = x(i, 1); % xv
        y(i,3) = x(i, 3); % yv
        y(i,4) = x(i, 5); % alv
        y(i,5) = x(i, 7); % th1
        y(i,6) = x(i, 9); % th2
        y(i,7) = x(i, 2); % xvdot
        y(i,8) = x(i, 4); % yvdot
        y(i,9) = x(i, 6); % alvdot
        y(i,10) = x(i, 8); % th1dot
        y(i,11) = x(i, 10); % th2dot
        y(i,12) = xdot(2); % xvddot
        y(i,13) = xdot(4); % yvddot
        y(i,14) = xdot(6); % alvddot
        y(i,15) = xdot(8); % th1ddot
        y(i,16) = xdot(10); % th2ddot
        y(i,17) = F(1); % ux
        y(i,18) = F(2); % uy
        y(i,19) = F(3); % t0
        y(i,20) = F(4); % t1
        y(i,21) = F(5); % t2
        y(i,22) = Xd; % X end desired
        y(i,23) = Yd; % Y end desired
        y(i,24) = ctrlParams.refx;
        y(i,25) = ctrlParams.refy;
        y(i,26) = ctrlParams.phi;
        y(i,27) = Xv; % desired vehicle x position
        y(i,28) = Yv; % desired vehicle x position
        y(i,29) = Alv; % desired vehicle pitch angle
        y(i,30) = Th1; % Th1 desired
        y(i,31) = Th2; % Th2 desired 
        if ctrlParams.noise
            y(i,32:41) = xm';
        end
    end
    if exist("noise.mat", 'file') == 2
        % delete("noise.mat")
    end
end

function [ts, xs] = select_samples(ctrlParams, t, x)
    switch ctrlParams.method
        case "random"
            indices = randperm(length(t), ctrlParams.numPoints);
            sortIndices = sort(indices);
            ts = t(sortIndices);
            xs = x(sortIndices,:);
        case "interval"
            ts = [t(1)];
            xs = [x(1,:)];
            for i = 2:length(t)
                if t(i)-ts(end) >= ctrlParams.interval
                    ts = [ts;t(i)];
                    xs = [xs;x(i,:)];
                end
            end
        otherwise
            ts = t;
            xs = x;
    end
end

function status = myOutputFcn(t, y, flag, startTime, stopTime)
    currentTime = datetime;
    status = double(seconds(currentTime-startTime) > stopTime);
end

function params = params_control()    
    params = struct();
    % PID gains for each input (Kp, Ki, Kd)
    params.PID1 = [10 10 10]; 
    params.PID2 = [10 10 10]; 
    params.PID3 = [10 10 10];
    params.PID4 = [10 10 10];
    params.PID5 = [10 10 10];
    params.refx = 1; % center of reference trajectory in x
    params.refy = 1; % center of reference trajectory in y
    params.xrange = 4; % width of reference point range
    params.yrange = 4; % height of reference point range
    params.a = 1; % target object horizontal dimension
    params.b = 0.5; % vertical dimension
    params.phi = 0; % angle of target point
    params.refrad = 0.5; % radius of reference trajectory 
    params.friction = "andersson"; % none, smooth, andersson, specker
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "random"; % random, interval, origin
    params.numPoints = 500;
    params.interval = 1e-3;
    params.solver = "nonstiff"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstiff"
end
function params = params_control()    
    params = struct();
    % PID gains for each input (Kp, Ki, Kd)
    controller = load("best_controller5.mat");
    params.PID1 = controller.BestChrom(1:3); %[3895.78697749516	1135.11211587263	4846.27078124992];
    params.PID2 = controller.BestChrom(4:6);%[7201.13284314069	2240.23687691478	8423.01078728416]; 
    params.PID3 = controller.BestChrom(7:9);%[6770.52751638764	5134.98667058038	5736.73346456017]; 
    params.PID4 = controller.BestChrom(10:12);%[1727.95110585053	6989.59605494114	675.792507516856]; 
    params.PID5 = controller.BestChrom(13:15);%[1727.95110585053	6989.59605494114	675.792507516856]; 
    params.Flim = 1000;
    params.Tlim = 50;
    params.Pf = load("best_controller4.mat").BestChrom(16);
    params.refx = 1; % center of reference trajectory in x
    params.refy = 1; % center of reference trajectory in y
    params.xrange = 4; % width of reference point range
    params.yrange = 4; % height of reference point range
    params.a = 0.5; % target object horizontal dimension
    params.b = 0.25; % vertical dimension
    params.phi = 0; % angle of target point
    params.noise = 1;
    params.sigma = 2e-2;
    params.refrad = 0.5; % radius of reference trajectory 
    params.friction = 0; 
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "random"; % random, interval, origin
    params.numPoints = 500;
    params.interval = 1e-3;
    params.solver = "nonstifflr"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstiff"
end
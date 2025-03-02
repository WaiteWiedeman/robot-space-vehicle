function params = params_system()
    params = struct();
    params.b1 = 1;
    params.b2 = 0.75;
    params.L1 = 0.5; % length of link 1
    params.L2 = 0.5; % length of link 2
    params.G = 0; %9.8; % gravity
    params.M0 = 300;  % point mass of cart
    params.M1 = 15; % point mass of link 1
    params.M2 = 15; % point mass of link 2
    params.mu_s = 0.15; % static friction 
    params.mu_k = 0.1; % kinetic friction
end
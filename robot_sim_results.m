%% clear workspace
close all; clear; clc;

%% Two-link Planar Robot on Cart Dynamics System Parameters
sysParams = params_system();
ctrlParams = params_control();
ctrlParams.method = "origin";
ctrlParams.solver = "stiffhr"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstiff"
ctrlParams.PID1 = [10000 50 500];
ctrlParams.PID2 = [10000 50 500];
ctrlParams.PID3 = [2000 50 500];
ctrlParams.PID4 = [2000 50 500];
ctrlParams.PID5 = [2000 50 500];
tSpan = 0:0.01:15; %[0,20]; %

%% run simulation and plot states, forces, and states against reference
% theta = 2*pi*rand;
% rad = sqrt(rand);
ctrlParams.refx = 1; %ctrlParams.xrange*rad*cos(theta);
ctrlParams.refy = 1; %ctrlParams.yrange*rad*sin(theta);
ctrlParams.phi = 2*pi*rand;
ctrlParams.a = 0.25+rand*0.25; % target object horizontal dimension
ctrlParams.b = 0.25+rand*0.25; % vertical dimension
% x0 = [-1; 0; 0] + [2; 2*pi; 2*pi].*rand(3,1); % th0, th1, th2
% x0 = [x0(1); 0; x0(2); 0; x0(3); 0]; % th0, th0d, th1, th1d, th2, th2d
x0 = zeros(10,1); % th0, th0d, th1, th1d, th2, th2d

y = robot_simulation(tSpan, x0, sysParams, ctrlParams);

% plot states, forces, and states against reference
plot_states(y(:,1),y(:,2:16),"position",y(:,27:31));
plot_states(y(:,1),y(:,2:16),"velocity",y(:,27:31));
plot_states(y(:,1),y(:,2:16),"acceleration",y(:,27:31));
plot_forces(y(:,1),y(:,17:21));

% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:6),sysParams);
plot_endeffector([xend yend],y(:,22:23)) %y(:,15:16)

info1 = lsiminfo(y(:,2),y(:,1));
info2 = lsiminfo(y(:,3),y(:,1));
info3 = lsiminfo(y(:,4),y(:,1));

MakeVideo(ctrlParams,sysParams, y(:,1), y(:,2:6), y(:,22:23), [0,tSpan(end)]);

% disp(xend(end))
% disp(yend(end))
% disp(y(end,2)-y(end,19))
% disp(y(end,3)-y(end,17))
% disp(y(end,4)-y(end,18))

%% run simscape model and plot states, forces, and states against reference for simscape model
y_simscape = run_simscape();

plot_states(y_simscape(:,1),y_simscape(:,2:10));
plot_forces(y_simscape(:,1),y_simscape(:,11),y_simscape(:,12),y_simscape(:,13),y_simscape(:,14));
plot_reference(y_simscape(:,1),y_simscape(:,2:4),y_simscape(:,15:18))

% solve forward kinematics and plot end effector position for simscape model
[~,~,~,~,~,~,xend,yend] = ForwardKinematics(y_simscape(:,2:4),sysParams);
plot_endeffector([xend yend],y_simscape(:,15:16)) %y(:,15:16)

%% Tune PID w/ GA
N_monte_carlo = 100;
myObj = @(gene) fitnessfun(gene, N_monte_carlo,ctrlParams,sysParams);
% GA parameters
Pc = 0.85;
fitfun = @(x) myObj(x);
PopSize = 200;
MaxGens = 100;
nvars   = 9;
A       = [];
b       = [];
Aeq     = [];               
beq     = [];
lb      = 100*ones(1,9);
ub      = 1000*ones(1,9);
nonlcon = [];
options = optimoptions('ga', 'PopulationSize', PopSize, 'MaxGenerations',...
    MaxGens,'PlotFcn',{@gaplotbestf,@gaplotscores});
options.CrossoverFraction = Pc;
% GA to search for near-optimal solution
start = datetime
[BestChrom, fval, exitflag, output] = ga(fitfun, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);
stop = datetime
disp(stop-start)

%% simulate PID controller tuned by GA
ctrlParams.PID0 = BestChrom(1:3); 
ctrlParams.PID1 = BestChrom(4:6);
ctrlParams.PID2 = BestChrom(7:9);

%% coulomb friction
v = linspace(-5,5,10000);
fc = coulomb_friction(v, sysParams, ctrlParams.friction);
idx = find(v >= 0, 1, 'first');
slopeangle = atan((fc(idx))/(v(idx)))*180/pi;
figure('Position',[500,100,800,800]);
plot(v,fc,'b-','LineWidth',2);
xlabel("velocity (m/s)");
ylabel('Coulomb Friction Force (N)');
set(gca, 'FontSize', 15);
set(gca, 'FontName', 'Arial');
disp(slopeangle)

%% random points
t=0:0.01:5;
x = ctrlParams.a*cos((2*pi/5)*t);
y = ctrlParams.b*sin((2*pi/5)*t);

numCase = 100;
theta = linspace(0,2*pi,numCase);
rad = linspace(0,1,numCase);

for i = 1:numCase
    % theta = 2*pi*rand;
    % rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.a*rad(i)*cos(theta(i));
    ctrlParams.refy = ctrlParams.b*rad(i)*sin(theta(i));
    Xd(i) = ctrlParams.refx;
    Yd(i) = ctrlParams.refy;
end

figure('Position',[500,100,800,800]);
plot(x,y,'k-',Xd,Yd,'b*','LineWidth',1)
axis padded
ylabel("Y");
xlabel("X");
set(get(gca,'ylabel'),'rotation',0);
set(gca, 'FontSize', 15);
set(gca, 'FontName', 'Arial');
legend("Objective Bounds", "Random Objective Point")

%% rotating ellipse
time = linspace(0,5,10);
th = linspace(0,2*pi,100);
beta = (2*pi/5)*time; %[pi/4, pi/2, 3*pi/4]; %linspace(0,2*pi,10);
h = 1; k = 1; a = 4; b = 3;
figure;
for i = 1:length(beta)
    x = h + a*cos(th)*cos(beta(i)) - b*sin(th)*sin(beta(i)); 
    y = k + b*sin(th)*cos(beta(i)) + a*cos(th)*sin(beta(i));
    plot(x,y,'o-')
    axis square
    hold on
end

%%
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,2),'b-o','LineWidth',2);
ylabel("$\theta_0$","Interpreter","latex");
set(get(gca,'ylabel'),'rotation',0);
set(gca, 'FontSize', 15);
set(gca, 'FontName', "Arial")
xlabel("Time (s)");

%% functions
function plot_forces(t,u)
    figure('Position',[500,100,800,800]);
    plot(t,u(:,1),'k-',t,u(:,2),'b-',t,u(:,3),'g-',t,u(:,4),'m-',t,u(:,5),'c-','LineWidth',2);
    legend("$u_x$","$u_y$","$\tau_0$","$\tau_1$","$\tau_2$","Interpreter","latex");
end

function plot_states(t,x,flag,refs)
labels= ["$x_v$","$y_v$","$\alpha_v$","$\theta_1$","$\theta_2$","$\dot{x}_v$","$\dot{y}_v$","$\dot{\alpha}_v$","$\dot{\theta}_1$","$\dot{\theta}_2$",...
    "$\ddot{x}_v$","$\ddot{y}_v$","$\ddot{\alpha}_v$","$\ddot{\theta}_1$","$\ddot{\theta}_2$"];
switch flag
    case "position"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 1:numState-10
            nexttile
            plot(t,x(:,i),'b-',t,refs(:,i),'k:','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-10
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Reference","Location","eastoutside","FontName","Arial");
    case "velocity"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 6:numState-5
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-5
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Location","eastoutside","FontName","Arial");
    case "acceleration"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 11:numState
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
end
end

function plot_endeffector(x,refs)
    refClr = "blue";
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    plot(x(:,1),x(:,2),'b-o','LineWidth',2);
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'*','LineWidth',2);
    end
    axis padded
    % xline(1,'k--','LineWidth',2);
    ylabel("Y");
    xlabel("X");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');

end

function [fit]  = fitnessfun(gene, N_monte_carlo,tSpan,ctrlParams,sysParams) %,Rise_thresh ,POlim,Ts_lim

ctrlParams.PID0 = gene(1:3);
ctrlParams.PID1 = gene(4:6);
ctrlParams.PID2 = gene(7:9);

% ts1_above_thresh = zeros(1,N_monte_carlo);
% ts2_above_thresh = zeros(1,N_monte_carlo);
% ts3_above_thresh = zeros(1,N_monte_carlo);
% ts4_above_thresh = zeros(1,N_monte_carlo);
% ts5_above_thresh = zeros(1,N_monte_carlo);
% x1_above_thresh = zeros(1,N_monte_carlo);
% x2_above_thresh = zeros(1,N_monte_carlo);
% x3_above_thresh = zeros(1,N_monte_carlo);
% x4_above_thresh = zeros(1,N_monte_carlo);
% x5_above_thresh = zeros(1,N_monte_carlo);
% rs1_above_thresh = zeros(1,N_monte_carlo);
% rs2_above_thresh = zeros(1,N_monte_carlo);
% rs3_above_thresh = zeros(1,N_monte_carlo);
% rs4_above_thresh = zeros(1,N_monte_carlo);
% rs5_above_thresh = zeros(1,N_monte_carlo);
e1 = zeros(1,N_monte_carlo);
e2 = zeros(1,N_monte_carlo);
e3 = zeros(1,N_monte_carlo);
e4 = zeros(1,N_monte_carlo);
e5 = zeros(1,N_monte_carlo);
exend = zeros(1,N_monte_carlo);
eyend = zeros(1,N_monte_carlo);

for sim_n = 1:N_monte_carlo
    theta = 2*pi*rand;
    rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.xrange*rad*cos(theta);
    ctrlParams.refy = ctrlParams.yrange*rad*sin(theta);
    ctrlParams.phi = 2*pi*rand;
    ctrlParams.a = 0.25+rand*0.25; % target object horizontal dimension
    ctrlParams.b = 0.25+rand*0.25; % vertical dimension
    % x0 = [-1; 0; 0] + [2; 2*pi; 2*pi].*rand(3,1); % th0, th1, th2
    % x0 = [x0(1); 0; x0(2); 0; x0(3); 0]; % th0, th0d, th1, th1d, th2, th2d
    x0 = zeros(10,1); % th0, th0d, th1, th1d, th2, th2d
    y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
    [~,~,~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:6),sysParams);
    
    if y(end,1) < tSpan(2)
        disp("loop bailed")
        break
    end

    e1(sim_n) = abs(mean(y(end-100:end,2) - y(end-100:end,27),'all'));
    e2(sim_n) = abs(mean(y(end-100:end,3) - y(end-100:end,28),'all'));
    e3(sim_n) = abs(mean(y(end-100:end,4) - y(end-100:end,29),'all'));
    e4(sim_n) = abs(mean(y(end-100:end,5) - y(end-100:end,30),'all'));
    e5(sim_n) = abs(mean(y(end-100:end,6) - y(end-100:end,31),'all'));
    exend(sim_n) = abs(mean(y(end-100:end,22) - xend(end-100:end),'all'));
    eyend(sim_n) = abs(mean(y(end-100:end,23) - yend(end-100:end),'all'));

    % info1 = lsiminfo(y(:,2),y(:,1));
    % info2 = lsiminfo(y(:,3),y(:,1));
    % info3 = lsiminfo(y(:,4),y(:,1));
    % info4 = lsiminfo(y(:,5),y(:,1));
    % info5 = lsiminfo(y(:,6),y(:,1));
    % 
    % Ts1 = info1.SettlingTime;
    % Ts2 = info2.SettlingTime;
    % Ts3 = info3.SettlingTime;
    % Ts4 = info4.SettlingTime;
    % Ts5 = info5.SettlingTime;
    % 
    % if x0(1) >= y(1,27)
    %     max1 = info1.Min;
    % else 
    %     max1 = info1.Max;
    % end
    % if x0(3) >= y(1,28)
    %     max2 = info2.Min;
    % else 
    %     max2 = info2.Max;
    % end
    % if x0(5) >= y(1,29)
    %     max3 = info3.Min;
    % else
    %     max3 = info3.Max;
    % end
    % if x0(7) >= y(1,30)
    %     max4 = info4.Min;
    % else
    %     max4 = info4.Max;
    % end
    % if x0(9) >= y(1,31)
    %     max5 = info5.Min;
    % else
    %     max5 = info5.Max;
    % end
    % 
    % if Ts1 > Ts_lim  
    %     ts1_above_thresh(sim_n) = 1;
    % elseif Ts2 > Ts_lim
    %     ts2_above_thresh(sim_n) = 1;
    % elseif Ts3 > Ts_lim
    %     ts3_above_thresh(sim_n) = 1;
    % elseif Ts4 > Ts_lim
    %     ts4_above_thresh(sim_n) = 1;
    % elseif Ts5 > Ts_lim
    %     ts5_above_thresh(sim_n) = 1;
    % end
    % 
    % if abs(max1 - y(end,19))/abs(x0(1) - y(end,19)) > POlim 
    %     x1_above_thresh(sim_n) = 1;
    % elseif abs(max2 - y(end,17))/abs(x0(3) - y(end,17)) > POlim
    %     x2_above_thresh(sim_n) = 1;
    % elseif abs(max3 - y(end,18))/abs(x0(5) - y(end,18)) > POlim
    %     x3_above_thresh(sim_n) = 1;
    % elseif abs(max4 - y(end,17))/abs(x0(3) - y(end,17)) > POlim
    %     x2_above_thresh(sim_n) = 1;
    % elseif abs(max5 - y(end,18))/abs(x0(5) - y(end,18)) > POlim
    %     x3_above_thresh(sim_n) = 1;
    % end
end
% disp('monte carlo done')
% disp(sum(x_above_thresh)/N_monte_carlo)
% disp(sum(settling_time_above_thresh)/N_monte_carlo)
% disp(sum(u_above_thresh)/N_monte_carlo)
% disp(mean(e1))
% disp(mean(e2))
% disp(mean(e3))
% disp(mean(exend))
% disp(mean(eyend))
% disp(Ts1)
% disp(Ts2)
% disp(Ts3)

% fitness is weighted squared probability
% fit = (1e2)*((sum(x_above_thresh)/N_monte_carlo)^2 + (sum(settling_time_above_thresh)/N_monte_carlo)^2 ...
%     + (sum(u_above_thresh)/N_monte_carlo)^2 + 10*mean(e1) + 10*mean(e2) + 10*mean(e3) + 10*mean(exend) + 10*mean(eyend)); % 
fit = 1e3*(e1+e2+e3+e4+e5+exend+eyend);
end

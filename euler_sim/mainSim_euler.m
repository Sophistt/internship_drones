clc; clear; close all;
addpath(genpath(pwd)); % Add files;

% trajectory generator
trajhandle = @trajCircle;

% controller
controlhandle = @innerloopControllers;

% real-time 
real_time = false;

% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********
% number of quadrotors
nquad = 1;

% max time
time_tol = 100;

% parameters for simulation
params = crazyfile();

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors
for qn = 1:nquad
    % Get start and stop position
    %des_start = trajhandle(0, qn);
    %des_stop  = trajhandle(inf, qn);
    stop{qn}  = [100;100;100]
    x0{qn}    = zeros(12, 1);
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

x         = x0;        % state
pos_tol   = 0.01;
vel_tol   = 0.01;


%% ************************* RUN SIMULATION *************************
OUTPUT_TO_VIDEO = 0;
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('diamond.avi');
    open(v)
end

fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter
    iter;
    timeint = time:tstep:time+cstep;

    tic;
    % Iterate over each quad
    for qn = 1:nquad
        % Run simulation
        [tsave, xsave] = ode45(@(t, y) droneSystem_euler(t, y, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn}    = xsave(end, :)';
        
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    end
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end
    
    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end


%% visualization
figure;
subplot(3, 1, 1);
plot(ttraj{1}, xtraj{1}(:, 1), 'Linewidth', 1.5);
xlabel("Time [s]")
ylabel("Position [m]")
title("X-axis")
grid on;

subplot(3, 1, 2);
plot(ttraj{1}, xtraj{1}(:, 2), 'Linewidth', 1.5);
xlabel("Time [s]")
ylabel("Position [m]")
title("Y-axis")
grid on;

subplot(3, 1, 3);
plot(ttraj{1}, xtraj{1}(:, 3), 'Linewidth', 1.5);
xlabel("Time [s]")
ylabel("Position [m]")
title("Z-axis")
grid on;

figure;
plot3(xtraj{1}(:,1), xtraj{1}(:,2), xtraj{1}(:,3), 'Linewidth', 1.5);
xlabel("Position X [m]")
ylabel("Position Y [m]")
zlabel("Position Z [m]")
grid on;


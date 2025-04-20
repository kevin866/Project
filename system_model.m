clc;
clear;

% Simulation parameters
T = 50;                       % Number of time steps
E = zeros(1, T+1);            % Energy in battery
E(1) = 4;                     % Initial battery energy (in kWh)

E_min = 1;                    % Battery minimum capacity
E_max = 6;                    % Battery maximum capacity
E_set = 3;                    % Desired set point (not used in this sim)

% Simulated energy flows
Es = 0.5 + 0.3*randn(1, T)   % Input energy (from grid/solar), some noise
El = 0.4 + 0.2*randn(1, T);   % Disturbance (load), random

% Clip negative values to avoid unrealistic behavior
Es = max(Es, 0);
El = max(El, 0);

% Simulate system dynamics
for k = 1:T
    E(k+1) = E(k) + Es(k) - El(k);
    
    % Enforce battery limits
    if E(k+1) > E_max
        E(k+1) = E_max;
    elseif E(k+1) < E_min
        E(k+1) = E_min;
    end
end

% Plot results
figure;
subplot(3,1,1);
plot(0:T, E, 'b', 'LineWidth', 2); hold on;
yline(E_min, '--k'); yline(E_max, '--k'); yline(E_set, '--r');
xlabel('Time Step'); ylabel('Energy (kWh)');
title('Battery Energy Level');

subplot(3,1,2);
plot(1:T, Es, 'g', 'LineWidth', 2);
xlabel('Time Step'); ylabel('Input Es (kWh)');
title('Input Energy from Grid/Solar');

subplot(3,1,3);
plot(1:T, El, 'm', 'LineWidth', 2);
xlabel('Time Step'); ylabel('Load El (kWh)');
title('Energy Drawn by Load (Disturbance)');

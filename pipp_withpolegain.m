clear all
A = 1; B = 1; C = 1;
A_aug = [A, 0; -C, 1];
B_aug = [B; 0];

desired_poles = [0.7, 0.6];
K = acker(A_aug, B_aug, desired_poles);
Kp = K(1);   % Proportional gain
Ki = K(2);   % Integral gain
% Simulation Parameters
k_max = 100;                  % simulation steps
Eset = 3;                     % desired battery level

% Disturbance (load), random (same for all runs to ensure fair comparison)
rng(0);  % Set seed for reproducibility
El = 1 + 0.1 * randn(1, k_max);   

fprintf('Kp = %.4f\nKi = %.4f\n', Kp, Ki);
% Initialize variables for each run
E = zeros(1, k_max);     
Es = zeros(1, k_max);    
E(1) = 2;                % initial battery level
integral_error = 0;

% PI Control Simulation
for k = 1:k_max-1
    e = Eset - E(k);
    integral_error = integral_error + e;
    Es(k) = Kp * e + Ki * integral_error;
    E(k+1) = E(k) + Es(k) - El(k);
end

% Plot each trajectory
plot(1:k_max, E, 'LineWidth', 1.5);
% legend_entries{end+1} = sprintf('Kp=%.2f, Ki=%.2f', Kp, Ki);

% Setpoint line
yline(Eset, '--r', 'Setpoint', 'LineWidth', 1.5);

% Final plot formatting
xlabel('Time step (k)');
ylabel('Battery Energy E(k) [kWh]');
title('Battery Energy Regulation for Various PI Gains');
% legend(legend_entries, 'Location', 'best');
grid on;
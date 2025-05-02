clear
% Simulation Parameters
k_max = 100;           % simulation steps
E = zeros(1, k_max);   % battery energy
Es = zeros(1, k_max);  % control input
El = 0.2*ones(1, k_max); % load (disturbance), assumed constant
El = -1+0.1*randn(1, k_max);   % Disturbance (load), random

% Initial Conditions
E(1) = 2;               % initial battery level
Eset = 3;               % desired battery level

% Controller Gains (you may tune these)
Kp = 0.5;
Ki = 0.05;

% PI terms
e_prev = 0;
u_prev = 0;
integral_error = 0;

for k = 1:k_max-1
    % Error
    e = Eset - E(k);
    integral_error = integral_error + e;

    % PI controller
    Es(k) = Kp * e + Ki * integral_error;

    % Apply control input to system
    E(k+1) = E(k) + Es(k) - El(k);

    % % Enforce battery constraints
    % if E(k+1) < 1
    %     E(k+1) = 1;
    % elseif E(k+1) > 6
    %     E(k+1) = 6;
    % end
end

% Plot
figure;
plot(1:k_max, E, 'b', 'LineWidth', 2);
hold on;
yline(Eset, '--r', 'Setpoint');
xlabel('Time step (k)');
ylabel('Battery Energy E(k) [kWh]');
title('Battery Energy Regulation with PI Control');
legend('E(k)', 'Setpoint');
grid on;

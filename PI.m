clear
% Simulation Parameters
k_max = 100;                  % simulation steps
Eset = 3;                     % desired battery level

% Disturbance (load), random (same for all runs to ensure fair comparison)
rng(0);  % Set seed for reproducibility
El = 1 + 0.1 * randn(1, k_max);   

% Gain values to test
Kp_values = [0.1, 1.0];
Ki_values = [0.01, 0.05];

% Prepare plot
figure;
hold on;

legend_entries = {};

% Loop through combinations of Kp and Ki
for i = 1:length(Kp_values)
    for j = 1:length(Ki_values)
        % Initialize variables for each run
        E = zeros(1, k_max);     
        Es = zeros(1, k_max);    
        E(1) = 2;                % initial battery level
        integral_error = 0;

        % Controller gains
        Kp = Kp_values(i)
        Ki = Ki_values(j)
        A_cl = [1 - Kp, Ki; -1, 1];
        eigvals = eig(A_cl)
        is_stable = all(abs(eigvals) < 1);
        disp(is_stable);


        % PI Control Simulation
        for k = 1:k_max-1
            e = Eset - E(k);
            integral_error = integral_error + e;
            Es(k) = Kp * e + Ki * integral_error;
            E(k+1) = E(k) + Es(k) - El(k);
        end

        % Plot each trajectory
        plot(1:k_max, E, 'LineWidth', 1.5);
        legend_entries{end+1} = sprintf('Kp=%.2f, Ki=%.2f', Kp, Ki);
    end
end

% Setpoint line
yline(Eset, '--r', 'Setpoint', 'LineWidth', 1.5);

% Final plot formatting
xlabel('Time step (k)');
ylabel('Battery Energy E(k) [kWh]');
title('Battery Energy Regulation for Various PI Gains');
legend(legend_entries, 'Location', 'best');
grid on;

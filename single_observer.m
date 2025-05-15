% Initialize
N = 50;                     % Number of time steps
E_true = zeros(1, N);       % True energy in battery
E_est = zeros(1, N);        % Estimated energy
E_meas = zeros(1, N);       % Measured energy (with noise)
Es = 0.2 * ones(1, N);      % Control input (solar + grid)
El = 0.4 +  0.5*randn(1, N);  % Unmeasured disturbance (load)

% Initial conditions
E_true(1) = 3.0;           % True initial energy
E_est(1) = 2.5;            % Initial guess
L = 0.3;                   % Observer gain (tune as needed)

% Simulation loop
for k = 1:N-1
    % True system dynamics
    E_true(k+1) = E_true(k) + Es(k) - El(k);
    
    % Measurement (with noise)
    E_meas(k) = E_true(k) + 0.01*randn;
    
    % Observer update
    E_est(k+1) = E_est(k) + Es(k) + L * (E_meas(k) - E_est(k));
end

% Plot
figure;
plot(1:N, E_true, 'b-', 'LineWidth', 2); hold on;
plot(1:N, E_est, 'r--', 'LineWidth', 2);
legend('True E(k)', 'Estimated E(k)');
xlabel('Time step');
ylabel('Energy (kWh)');
title('State Observer (E only, ignoring El)');
grid on;

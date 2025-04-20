% Time steps
N = 50;

% Setpoint and limits
Emin = 1; Emax = 6; Eset = 3;

% Preallocate storage
x_hat = zeros(2, N);     % Estimated states [E; El]
P = eye(2);              % Initial error covariance
Q = diag([0.01, 0.1]);   % Process noise covariance
R = 0.01;                % Measurement noise covariance

% System matrices
A = [1 -1; 0 1];
B = [1; 0];
C = [1 0];

% Initial state
x_hat(:,1) = [3; 0];  % Assume starting at E = 3 kWh, El unknown

% Generate input (Es) and measurement (y)
Es = 0.5 + 0.2*randn(1, N);          % Input energy (can replace with real data)
true_El = 0.4 + 0.2*randn(1, N);    % True (unknown) discharge
E_true = zeros(1, N);               % True energy in battery
y = zeros(1, N);                    % Measured energy (with noise)

E_true(1) = x_hat(1,1);
y(1) = E_true(1) + sqrt(R)*randn;

% Simulate true system for generating synthetic measurement
for k = 2:N
    E_true(k) = E_true(k-1) + Es(k-1) - true_El(k-1);
    y(k) = E_true(k) + sqrt(R)*randn;
end
for k = 2:N
    % Predict step
    x_pred = A * x_hat(:,k-1) + B * Es(k-1);
    P_pred = A * P * A' + Q;

    % Kalman Gain
    K = P_pred * C' / (C * P_pred * C' + R);

    % Update step
    x_hat(:,k) = x_pred + K * (y(k) - C * x_pred);
    P = (eye(2) - K * C) * P_pred;
end
figure;
subplot(3,1,1);
plot(1:N, E_true, 'g', 1:N, x_hat(1,:), 'b--');
legend('True E', 'Estimated E');
title('Battery Energy Storage');

subplot(3,1,2);
plot(1:N, true_El, 'r', 1:N, x_hat(2,:), 'k--');
legend('True El', 'Estimated El');
title('Discharge Energy Estimation');

subplot(3,1,3);
plot(1:N, Es);
title('Input Energy Es');
xlabel('Time Step');

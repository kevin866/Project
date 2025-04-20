clear all
A = [1 -1; 0 1];
B = [1; 0];
C = [1 0];
D = 0;

sys = ss(A, B, C, D, 1);  % 1 = sampling time (discrete system)
Q = diag([0.01, 0.1]);  % Process noise covariance
R = 0.01;               % Measurement noise covariance
N = 0;                  % Cross-covariance (assumed zero)
[kalmf, L, P, M] = kalman(sys, Q, R, N);
% Simulation settings
N_sim = 50;
Es = 0.5 + 0.2*randn(N_sim, 1);        % Input energy
true_El = 0.3 + 0.05*randn(N_sim, 1);  % True disturbance
E_true = zeros(N_sim, 1);
y = zeros(N_sim, 1);

% Initial energy
E_true(1) = 3;
y(1) = E_true(1) + sqrt(R)*randn;

% Simulate system
for k = 2:N_sim
    E_true(k) = E_true(k-1) + Es(k-1) - true_El(k-1);
    y(k) = E_true(k) + sqrt(R)*randn;
end

% Combine Es and y as inputs to estimator
inputs = [Es, y];

% Estimate states
[~, ~, x_hat] = lsim(kalmf, inputs, (0:N_sim-1)');

% x_hat(:,1): estimated E
% x_hat(:,2): estimated El
figure;
subplot(2,1,1);
plot(E_true, 'g', 'LineWidth', 1.5); hold on;
plot(x_hat(:,1), 'b--');
legend('True E', 'Estimated E');
title('Energy Storage');

subplot(2,1,2);
plot(true_El, 'r', 'LineWidth', 1.5); hold on;
plot(x_hat(:,2), 'k--');
legend('True El', 'Estimated El');
title('Discharge Energy Estimation');
xlabel('Time step');

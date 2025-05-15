N = 50;

E_true = zeros(1,N);
Es = 0.5 + 0*randn(1,N);
El = 0.5 + 0.2*randn(1,N);  % Gaussian noise

E_true(1) = 3;

y = zeros(1,N);
y(1) = E_true(1);

for k = 2:N
    E_true(k) = E_true(k-1) + Es(k-1) - El(k-1);
    y(k) = E_true(k) + 0.1*randn;
end

% Kalman Filter Initialization
x_hat = zeros(1,N);
P = 1;
Q = 0.2^2;    % From El variance
R = 0.01;

 % Item 1: System Simulation (Standard Model)
 E1 = zeros(1, N); E1(1) = 3;
 for k = 1:N-1
     E1(k+1) = E1(k) + Es1(k) - El(k);
     E1(k+1) = min(max(E1(k+1), Emin), Emax);
 end
x_hat(1) = 3;

for k = 2:N
    % Predict
    x_pred = x_hat(k-1) + Es(k-1) - 0.5;  % Subtract mean of El
    P_pred = P + Q;

    % Update
    K = P_pred / (P_pred + R);
    x_hat(k) = x_pred + K * (y(k) - x_pred);
    P = (1 - K) * P_pred;
end

% Plot
figure;
plot(1:N, E_true, 'g', 1:N, x_hat, 'b--');
legend('True E', 'Estimated E');
title('Battery Energy Estimation');

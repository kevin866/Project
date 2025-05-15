clc; clear;

T = 50;
E_true = zeros(1, T+1);     % True battery energy
El_true = 0.4 + 0.5*randn(1, T);  % True disturbance (unknown to observer)
Es = 0.5 + 0*randn(1, T);       % Input energy (known)
% Es =0.5+ 0 * randn(1, T); 
% Clip negative values
El_true = max(El_true, 0);
Es = max(Es, 0);

E_true(1) = 4;              % Initial true energy

% Augmented system
A = [1 -1; 0 1];
B = [1; 0];
C = [1 0];

%Check observablity
Q = check_obser(A,C);

% Observer design (you can also use place(A',C',...)' or dlqr)
% placing ploe at 0.5 0.6 within the unit circle for discrete system
L = place(A', C', [0.5 0.6])'  % Observer gain

% Initialize observer
x_hat = zeros(2, T+1);          % Estimated [E; El]
x_hat(:,1) = [4; 0.3];          % Initial guess

% Simulate true system and observer
for k = 1:T
    % True system
    E_true(k+1) = E_true(k) + Es(k) - El_true(k);

    % Observer
    y_hat = C * x_hat(:,k);
    y_measured = E_true(k);  % Assume we can measure E(k)

    % Observer state update
    x_hat(:,k+1) = A*x_hat(:,k) + B*Es(k) + L*(y_measured - y_hat);
end

% Plotting
time = 1:T;
figure;

subplot(2,1,1);
plot(0:T, E_true, 'b', 'LineWidth', 2); hold on;
plot(0:T, x_hat(1,:), '--r', 'LineWidth', 2);
xlabel('Time'); ylabel('Energy (kWh)');
title('Battery Energy: True vs Estimated');
legend('True E(k)', 'Estimated E(k)');
% 
subplot(2,1,2);
plot(time, El_true, 'm', 'LineWidth', 2); hold on;
plot(time, x_hat(2,1:T), '--k', 'LineWidth', 2);
xlabel('Time'); ylabel('Load Energy (kWh)');
title('Load Energy Drawn: True vs Estimated');
legend('True E_l(k)', 'Estimated E_l(k)');

function Q = check_obser(A,C)
    Q = [C; C*A] % Calculate observability matrix Q
    Q_rank = rank(Q);
    if (Q_rank == size(A,1)) % Logic to assess observability
    disp('System is observable.');
    else
    disp('System is NOT observable.');
    end
end

clear
% Parameters
N = 10;                  % Horizon
Emin = 1;                % Min energy
Emax = 6;                % Max energy
Eset = 3;                % Reference set point
Q = 1;                   % Output tracking penalty
R = 0.1;                 % Control effort penalty
P = 1;                   % Terminal cost
A = 1;
B = 1;
C = 1;

% Disturbance
El = 0.5 * ones(1, N);   % Unmeasured load over horizon
El = 0.4 + 0.2*randn(1, N);   % Disturbance (load), random

% Initialization
x = 2;                   % Initial battery energy
x_traj = zeros(1, N+1);  % Store trajectory
x_traj(1) = x;
u_traj = zeros(1, N);    % Store control input

% Pre-allocate matrices
S = zeros(1, N);
K = zeros(1, N);
Kv = zeros(1, N);
v = zeros(1, N+1);

% Terminal condition
SN = C' * P * C;
S(N) = SN;

% Reference trajectory
r = Eset * ones(1, N+1);

% Backward Riccati recursion
for k = N:-1:1
    Sk1 = S(:,:,k+1);
    BT_Sk1_B = B' * Sk1 * B;
    inv_term = inv(BT_Sk1_B + R);

    K(:,:,k) = inv_term * B' * Sk1 * A;
    Kv(:,:,k) = inv_term * B';

    Ak = A - B * K(:,:,k);
    S(:,:,k) = A' * Sk1 * Ak + C' * Q * C;
    v(:,k) = Ak' * v(:,k+1) + C' * Q * r(:,k);
end

% Forward simulation
for k = 1:N
    % Compute gains
    Kk = (B' * S(k) * B + R)^(-1) * B' * S(k) * A;
    Kvk = (B' * S(k) * B + R)^(-1) * B';

    % Compute control input
    u = -Kk * x + Kvk * v(k+1);

    % Enforce constraints (optional)
    x = x + u - El(k);
    x = min(max(x, Emin), Emax);

    % Store data
    x_traj(k+1) = x;
    u_traj(k) = u;
end

% Plot results
figure;
subplot(2,1,1);
plot(0:N, x_traj, '-o'); hold on;
yline(Eset, '--r'); ylim([0 7]);
title('Battery Energy');
xlabel('Time step'); ylabel('E (kWh)');

subplot(2,1,2);
stairs(0:N-1, u_traj, '-o');
title('Control Input E_s');
xlabel('Time step'); ylabel('E_s (kWh)');

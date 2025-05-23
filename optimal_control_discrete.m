% % System with Known Disturbance
% % Discrete Linear Quadratic Tracker
% Assuming the disturbance is known

% System dynamics
A = 1;
B = 1;
C = 1;  % Identity output matrix

% Horizon
N = 50;

% Cost matrices
Q = 10;         % Output tracking cost
R = 10;            % Control effort
P = 1;    % Terminal cost

% Allocate memory
nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);
S = zeros(nx, nx, N+1);
K = zeros(nu, nx, N);
Kv = zeros(nu, ny, N);
v = zeros(ny, N+1);

% Reference trajectory
r = zeros(ny, N+1);
for k = 1:N+1
    r(:,k) = 3;
end

% Terminal condition
S(:,:,N+1) = C' * P * C;
v(:,N+1) = C' * P * r(:,N+1);

% Backward Riccati recursion for S_k and v_k
for k = N:-1:1
    dk = 0.1+10*rand();
    Sk1 = S(:,:,k+1);
    BT_Sk1_B = B' * Sk1 * B;
    inv_term = inv(BT_Sk1_B + R);
    K(:,:,k) = inv_term * B' * Sk1 * A;
    Kv(:,:,k) = inv_term * B';

    % Update S_k
    Ak = A - B * K(:,:,k);
    S(:,:,k) = A' * Sk1 * Ak + C'* Q * C;

    % Update v_k
    v(:,k) = Ak' * v(:,k+1) - Ak' * Sk1 * dk + C'*Q*r(:,k);
end

% Initial condition
x = zeros(nx, N+1);
u = zeros(nu, N);

x(:,1) = 2;

% Forward simulation
for k = 1:N
    u(:,k) = -K(:,:,k) * x(:,k) + Kv(:,:,k) * v(:,k+1);
    x(:,k+1) = A * x(:,k) + B * u(:,k);
end

% Plotting
figure;
subplot(2,1,1);
plot(1:N+1, x(1,:), 'b', 1:N+1, r(1,:), 'r--');
ylabel('x_1 vs r_1');

subplot(2,1,2);
plot(1:N, u(1,:), 'k');
xlabel('Time step');
ylabel('Control input u');

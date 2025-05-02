function discrete_LQT_multiple_QR()
    % Define Q and R values to test
    Q_vals = [1 10 100];
    R_vals = [1];
    % Define colors for Q values and line styles for R values
    colors = {'b', 'g', 'm'};  % Q: 1, 10, 100
    lineStyles = {'-', '--', ':'};  % R: 0.1, 1, 10

    % Loop through Q and R combinations
    figure;
    for i = 1:length(Q_vals)
        for j = 1:length(R_vals)
            Q = Q_vals(i);
            R = R_vals(j);
            [x, u, r] = run_LQT(Q, R);  % Run the LQT

            style = [colors{i} lineStyles{j}];  % Combine color and linestyle

            % Plot state trajectory
            subplot(2,1,1);
            hold on;
            plot(1:length(x), x, style, 'LineWidth', 1.2, ...
                'DisplayName', ['Q=' num2str(Q) ', R=' num2str(R)]);

            % Plot control trajectory
            subplot(2,1,2);
            hold on;
            plot(1:length(u), u, style, 'LineWidth', 1.2, ...
                'DisplayName', ['Q=' num2str(Q) ', R=' num2str(R)]);
        end
    end

    subplot(2,1,1);
    plot(1:length(r), r, 'k--', 'LineWidth', 1, 'DisplayName', 'Reference');
    ylabel('x vs r');
    legend('Location', 'best');

    subplot(2,1,2);
    xlabel('Time step');
    ylabel('Control input u');
    legend('Location', 'best');

end

function [x_out, u_out, r_out] = run_LQT(Q, R)
    % System dynamics
    A = 1;
    B = 1;
    C = 1;  % Identity output matrix
    d = 10*randn(1);  % Known constant disturbance
    
    % Horizon
    N = 50;
    
         % Control effort
    P = 1;           % Terminal cost
    
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
    v(:,N+1) = C' * P * r(:,N+1);  % Initialize with tracking, if not tracking then v(:,N+1) = 0
    
    % Backward Riccati recursion for S_k and v_k (with disturbance)
    for k = N:-1:1
        d = 10*randn(1);  % Known constant disturbance
        Sk1 = S(:,:,k+1);
        BT_Sk1_B = B' * Sk1 * B;
        inv_term = inv(BT_Sk1_B + R);
        K(:,:,k) = inv_term * B' * Sk1 * A;
        Kv(:,:,k) = inv_term * B';
    
        % Update S_k
        Ak = A - B * K(:,:,k);
        S(:,:,k) = A' * Sk1 * Ak + C' * Q * C;
    
        % Update v_k with disturbance correction
        v(:,k) = Ak' * v(:,k+1) - Ak' * Sk1 * d + C' * Q * r(:,k);
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

    x_out = x(1,:);
    u_out = u(1,:);
    r_out = r(1,:);
end

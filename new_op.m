function LQT_with_disturbance_observer()
    % Augmented system
    A = [1 -1; 0 1];
    B = [1; 0];
    C = [1 0];
    
    % Design observer
    desired_poles = [0.8, 0.9];  % place observer poles inside unit circle
    L = place(A', C', desired_poles)';          % transpose for correct dimensions

    % LQT parameters
    Q = 10;       % tracking cost
    R = 1;            % control effort
    P = Q;            % terminal cost
    N = 200;           % horizon
    ref = 3 * ones(1, N+1);  % constant reference for E
    
    % Backward recursion for LQT gains
    S = zeros(2,2,N+1);
    v = zeros(2,N+1);
    K = zeros(1,2,N);
    Kv = zeros(1,2,N);
    S(:,:,N+1) = C'*P*C;
    v(:,N+1) = C'*P*ref(N+1);
    
    for k = N:-1:1
        Sk1 = S(:,:,k+1);
        K(:,:,k) = (B'*Sk1*B + R)\(B'*Sk1*A);
        Kv(:,:,k) = (B'*Sk1*B + R)\B';
        Abar = A - B*K(:,:,k);
        S(:,:,k) = A'*Sk1*Abar + C'*Q*C;
        v(:,k) = Abar'*v(:,k+1) + C'*Q*ref(k);
    end

    % Simulation setup
    x_true = zeros(2, N+1);   % true state: [E; E_l]
    x_hat  = zeros(2, N+1);   % observer estimate
    y      = zeros(1, N+1);   % measured output
    u      = zeros(1, N);     % control input
    
    x_true(:,1) = [2; 1];     % Initial true state (E=2, E_l=1)
    x_hat(:,1)  = [2; 0.3];     % Initial estimate

    for k = 1:N
        % Compute control using observer estimate
        u(k) = -K(:,:,k)*x_hat(:,k) + Kv(:,:,k)*v(:,k+1);

        % True system evolves with unknown E_l (which stays constant)
        x_true(:,k+1) = A*x_true(:,k) + B*u(k);
        x_true(2,k+1) = x_true(2,k+1) + 0.3 * randn();  % 0.1 is standard deviation

        % Measure E (output)
        y(k+1) = C*x_true(:,k+1);

        % Observer update
        x_hat(:,k+1) = A*x_hat(:,k) + B*u(k) + L*(y(k+1) - C*x_hat(:,k));
    end

    % Plot results
    figure;
    subplot(3,1,1);
    plot(0:N, x_true(1,:), 'b', 0:N, x_hat(1,:), 'r--');
    hold on; plot(0:N, ref, 'k:');
    ylabel('E and \hat{E}','Interpreter', 'latex');
    legend('E_{true}', '\hat{E}', 'reference');

    subplot(3,1,2);
    plot(0:N, x_true(2,:), 'g', 0:N, x_hat(2,:), 'm--');
    ylabel('E_l and \hat{E}_l','Interpreter', 'latex');
    legend('E_l (true)', '\hat{E}_l','Interpreter', 'latex');

    subplot(3,1,3);
    stairs(0:N-1, u, 'LineWidth', 1.5);
    xlabel('Time step'); ylabel('Control input u');
    legend('u(k)');
end

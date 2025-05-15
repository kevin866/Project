function LQT_with_disturbance_observer_multiQR()
    % Augmented system
    A = [1 -1; 0 1];
    B = [1; 0];
    C = [1 0];

    % Design observer
    desired_poles = [0.9, 0.9];  % place observer poles inside unit circle
    L = acker(A', C', desired_poles)';          % transpose for correct dimensions

    % Parameters
    Q_values = [1, 100];      % Different tracking costs
    R_values = [1, 10];      % Different control efforts
    P = 1;                        % Terminal cost
    N = 100;                       % Horizon
    ref = 3 * ones(1, N+1);       % Constant reference

    % Colors for plotting
    colors = lines(length(Q_values) * length(R_values));
    figure; 

    % Create subplots for all three plots
    subplot(3,1,1); hold on; title('E vs. time'); ylabel('E');
    subplot(3,1,2); hold on; title('E_l vs. time'); ylabel('E_l');
    subplot(3,1,3); hold on; title('Control input u vs. time'); ylabel('u');

    for i = 1:length(Q_values)
        for j = 1:length(R_values)
            Q = Q_values(i);
            R = R_values(j);
            label = sprintf('Q=%.1f, R=%.1f', Q, R);
            color_idx = (i - 1) * length(R_values) + j;

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

            % Simulation
            x_true = zeros(2, N+1);
            x_true(2,:) = 0.5+0.3*randn(1,N+1);
            x_hat  = zeros(2, N+1);
            y      = zeros(1, N+1);
            u      = zeros(1, N);

            x_true(1,1) = 2;
            x_hat(:,1)  = [2; 0.3];

            for k = 1:N
                u(k) = -K(:,:,k)*x_hat(:,k) + Kv(:,:,k)*v(:,k+1);
                x_true(1,k+1) = A(1,:)*x_true(:,k) + B(1)*u(k);
                
                % x_true(2,k+1) =0.5+ 0.3 * randn(1);
                y(k+1) = C*x_true(:,k+1);
                x_hat(:,k+1) = A*x_hat(:,k) + B*u(k) + L*(y(k+1) - C*x_hat(:,k));
            end

            subplot(3,1,1); plot(0:N, x_true(1,:), '-', 'DisplayName', label, 'Color', colors(color_idx,:));
            subplot(3,1,2); plot(0:N, x_true(2,:), '-', 'DisplayName', label, 'Color', colors(color_idx,:));
            subplot(3,1,3); plot(0:N-1, u, '-', 'DisplayName', label, 'Color', colors(color_idx,:));
        end
    end

    % Add reference line and legends
    subplot(3,1,1); plot(0:N, ref, 'k--', 'DisplayName', 'Reference');
    subplot(3,1,1); legend show; xlabel('Time step');
    
    subplot(3,1,2); legend show; xlabel('Time step');
    
    subplot(3,1,3); legend show; xlabel('Time step');
end

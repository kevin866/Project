% Project 2: MPC - Battery Energy Storage Control
 % Items 1 to 5 implemented and simulated separately with augmented system
 
 clear; clc; close all;
 
 % General Parameters
 Emin = 1; Emax = 6; Eset = 3;
 N = 100;
 t = 1:N;
 
 % Augmented system
 A_aug = [1 -1; 0 1];
 B_aug = [1; 0];
 C_aug = [1 0];
 
 % Disturbance and input
 El = 0.3 * randn(1, N);
 Es1 = 0.5 * randn(1, N);
 
 El = max(El, 0);
 Es1 = max(Es1, 0);
 
 % Item 1: System Simulation (Standard Model)
 E1 = zeros(1, N); E1(1) = 3;
 for k = 1:N-1
     E1(k+1) = E1(k) + Es1(k) - El(k);
     E1(k+1) = min(max(E1(k+1), Emin), Emax);
 end
 
 figure(1);
 subplot(5,1,1);
 plot(t, E1, 'b'); title('Item 1: Raw System Simulation');
 ylabel('E1 (kWh)');
 
 % Item 2: Observer Estimation using Augmented System
 if rank([C_aug;C_aug*A_aug]) == 2
     disp('Then the system is observable')
 else
     disp('Then the system is unobservable')
 end
 
 x_hat = zeros(2, N);           % Estimated [E; El]
 x_hat(:,1) = [3; 0];           % Initial guess
 
 % Observer gain ( poles at 0.6 and 0.7)
 L = place(A_aug', C_aug', [-0.6 -0.7])';
 
 for k = 1:N-1
     y_hat = C_aug * x_hat(:,k);
     y_measured = E1(k);
     x_hat(:,k+1) = A_aug * x_hat(:,k) + B_aug * Es1(k) + L * (y_measured - y_hat);
 end
 
 subplot(5,1,2);
 plot(t, El, 'r', t, x_hat(2,1:N), 'b--');
 title('Item 2: Observer Estimation (E_l)');
 legend('True El', 'Estimated El');
 ylabel('E_l');
 
 % Item 3: Kalman Filter (Augmented)
 x_kf = zeros(2, N); x_kf(:,1) = [3; 0];
 P = eye(2);
 Q = diag([0.01, 0.1]);
 R = 0.01;
 
 y_meas = E1 + sqrt(R)*randn(1,N); % simulate measurement noise
 
 for k = 2:N
     % Predict
     x_pred = A_aug * x_kf(:,k-1) + B_aug * Es1(k-1);
     P_pred = A_aug * P * A_aug' + Q;
 
     % Update
     K = P_pred * C_aug' / (C_aug * P_pred * C_aug' + R);
     x_kf(:,k) = x_pred + K * (y_meas(k) - C_aug * x_pred);
     P = (eye(2) - K * C_aug) * P_pred;
 end
 
 subplot(5,1,3);
 plot(t, E1, 'b', t, x_kf(1,1:N), 'g--');
 title('Item 3: Kalman Filter Estimate (E)');
 legend('Actual E', 'Estimated E');
 ylabel('E_k');
 
 % Item 4: Optimal Control using Augmented State
 E4 = zeros(1, N); E4(1) = 3;
 x4 = [3; 0];
 Es_opt = zeros(1, N);
 lambda = 0.1;
 
 for k = 1:N-1
     if k == 1
         deltaE = 0;
     elseif k == 2
         deltaE = Es_opt(k-1);
     else
         deltaE = Es_opt(k-1) - Es_opt(k-2);
     end
     Es_opt(k) = -0.5 * lambda * deltaE + 0.8 * (Eset - x4(1));
 
     % Simulate with real disturbance
     x4 = A_aug * x4 + B_aug * Es_opt(k);
     x4(1) = min(max(x4(1), Emin), Emax);
     x4(2) = El(k); % replace unmeasured El with true El for next step
     E4(k+1) = x4(1);
 end
 
 subplot(5,1,4);
 plot(t, E4, 'k');
 title('Item 4: Optimal Control Result');
 ylabel('E_opt');
 
 % Item 5: PI Controller using Estimated E only
 E5 = zeros(1, N); E5(1) = 3;
 Es5 = zeros(1, N); integral = 0;
 Kp = 0.8; Ki = 0.1;
 
 for k = 1:N-1
     error = Eset - x_kf(1,k); % Use estimated E
     integral = integral + error;
     Es5(k) = Kp * error + Ki * integral;
     Es5(k) = min(max(Es5(k), 0), Emax - E5(k));
     E5(k+1) = E5(k) + Es5(k) - El(k); % Use real El for sim
     E5(k+1) = min(max(E5(k+1), Emin), Emax);
 end
 
 subplot(5,1,5);
 plot(t, E5, 'g');
 title('Item 5: PI Controller Response (using estimated E)');
 ylabel('E_PI');
 xlabel('Time step');
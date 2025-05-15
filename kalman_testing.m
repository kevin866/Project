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
 El = 0.5+0.3 * randn(1, N);
 Es1 = 0.5 + 0*randn(1, N);
 
 El = max(El, 0);
 Es1 = max(Es1, 0);
 
 % Item 1: System Simulation (Standard Model)
 E1 = zeros(1, N); E1(1) = 3;
 for k = 1:N-1
     E1(k+1) = E1(k) + Es1(k) - El(k);
     E1(k+1) = min(max(E1(k+1), Emin), Emax);
 end

% Item 3: Kalman Filter (Augmented)
 x_kf = zeros(2, N); x_kf(:,1) = [3; 0];
 P = eye(2);
 Q = diag([1e-5, 0.9]);
 R = 1e-5;
 
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
 
 % subplot(5,1,3);
 plot(t, E1, 'b', t, x_kf(1,1:N), 'g--');
 title('Item 3: Kalman Filter Estimate (E)');
 legend('Actual E', 'Estimated E');
 ylabel('E_k');
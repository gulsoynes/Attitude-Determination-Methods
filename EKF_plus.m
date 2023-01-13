function [X,P] = EKF_plus(q_sing,Euler_ang_sing,w_angular,P_sing,N)
% Algorithm is written by,
%  Hajiyev, C., and Soken, H. E., Fault Tolerant Attitude Estimation for Small Satellites, CRC Press, 2020. https://doi.org/10.1201/
% 9781351248839.

% Satellite Characteristic
delt = .1; J_x = 2.1e-3;
J_y = 2e-3;
J_z = 1.9e-3; Nt = 3.6e-10;

P(1).a = 100 * eye(7);  %Initial Cov Matrix
H = eye(7); %Measurement Matrix

%Process noise covariance matrix
Q = 0.05 * eye(7);   %Better for angular vel. (Changeable for Opt.)

%INITIAL STATE from TRIAD
X(:,1) = [q_sing(:,1); w_angular(:,1) + 0.006*rand(3,1)];

for k = 1:N
    %EXTRAPOLATION - PREDICTION of STATE
    F = F_Matrix(X(:,k),delt,J_x,J_y,J_z); %Linearized Transition Matrix
    X(:,k+1) = True_States2(X(:,k),J_x,J_y,J_z,Nt,delt);
    
    %MEASUREMENT MODEL From TRIAD
    v = [0;0;0;0;.006; .006; .006] .* rand(7,1);
    z(:,k+1) = [q_sing(:,k+1); w_angular(:,k+1)] + v;
    
    %PREDICTION of P(k+1)
    P(k+1).a = F * P(k).a * F' + Q;
    
    %MEASUREMENT NOICE Covariance Matrix
    P_tr = P_sing(k+1).a;
    if length(P_tr) == 3
        R = R_Matrix(P_tr, Euler_ang_sing(:,k+1));
    else
        R = [P_tr zeros(4,3);
            zeros(3,4) diag([0.006 0.006 0.006])];
        %RE CONSTRUCTION of R MATRIX
        R = diag(R) .* eye(7,7);
    end
    %KALMAN GAIN FORMULA
    K = P(k+1).a * H' / (H * P(k+1).a * H' + R);
    %CORRECTION of P(k+1)
    P(k+1).a = (eye(7) - K * H) * P(k+1).a;
    %CORRECTION of X(k+1)
    X(:,k+1) = X(:,k+1) + K * (z(:,k+1) - H * X(:,k+1));
    %Gain(k+1).a = K;
end
end


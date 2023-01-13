%2022-2023 GRADUATION PROJECT : SINGLE FRAME and KALMAN FILTERING BASE
%METHODS for ATTITUDE DETERMINATION

%This code contains caculations of Earth's magnetic field, sun and nadir
%direction vectors in body and reference frame.
%Second part contains TRIAD method, q method and Kalman Filtering.

%%%Written by Neslihan Gülsoy

clear; clc; close all
tic
%Parameters for Earth
mu = 3.98601e14;        %Earth Gravitational Constant (m^3/s^2)

%Orbital Elements
h = 400e3;                      %Altitude of Satellite (m)
Re = 6378.140e3;                %Earth radius on Equator (m)
Ro = ( Re + h );                %Distance Between Satellite and Earth (m)
N_t = 3.6e-10;      %The disturbance torque acting on the satellite (N.m)

inc = 97.65;                       %Otbit Inclination (deg)
Omega = 207.4;                     %Right Ascension of the Ascending Node (deg)
omega = 10;                      %Argument of perigee (deg)
T = 2 * pi * sqrt(Ro^3 / mu);   %Period of the satellite (sec)

%Iteration Parameters
N = 55537;          %Iteration number
delt = .1;          %Sample time (s)
time = linspace (0 , N*delt , N+1);

%Initial Data of the Attitude Angles

q_true = ...
    [0.005;             %q1
    0.015;               %q2
    0.01;              %q3
    ];
q_true = [q_true; sqrt(1 - sum(q_true.^2))];

%Initial Data for Satellite’s Angular Velocities (rad/s)
w_angular = ...
    [.001;
    .0015;
    .001];

%The Initial Moments of Inertia of the Satellite (m^4)
J_x = 2.1e-3;
J_y = 2e-3;
J_z = 1.9e-3;

[q_true,w_angular,C,Euler_ang_true] = True_States(q_true,w_angular,J_x,J_y,J_z,N_t,delt,N);


%Parameters for Sensors
sigma_h = .008;     %The standard deviation of magnetometer error
sigma_s = .002;     %The standard deviation of each Sun sensor noise
sigma_n = .006;     %The standard deviation of each Horizon sensor noise

%Sensor Modelling
[H_o, H_b] = Magnetometer(Ro,inc,C,sigma_h,time,N);
[S_o, S_b] = SunSensor(omega,Omega,inc,T,C,sigma_s,time,N);
[N_o, N_b] = HorizonSensor(Re,Ro,omega,Omega,T,C,sigma_n,time,N);


%% Only Nadir and Magnetometer
str = 'Nadir-Magnetometer'
for i = 1:N+1
    
    sigma = [std(N_b(:,i)) + std(N_o(:,i));
        std(H_b(:,i)) + std(H_o(:,i))];
    sgn = sign(q_true(:,i));
     
    b = [N_b(:,i), H_b(:,i)];
    r = [N_o(:,i), H_o(:,i)];
    
    [q_TRIAD(:,i), P_Triad(i).a ] = TRIAD(r,b,sigma,sgn);
    Euler_ang_TRIAD(:,i) = qtoEuler(q_TRIAD(:,i));
   
    [q_qMethod(:,i), P_qMethod(i).a] = qMethod(r,b,sigma,sgn);
    Euler_ang_qMethod(:,i) = qtoEuler(q_qMethod(:,i));
    
end

%% Only Sun and Magnetometer
str = 'Sun-Magnetometer'
for i = 1:N+1
    
    sigma = [std(S_b(:,i)) + std(S_o(:,i));
        std(H_b(:,i)) + std(H_o(:,i))];
       
    sgn = sign(q_true(:,i));
    
    b = [S_b(:,i), H_b(:,i)];
    r = [S_o(:,i), H_o(:,i)];
    
    [q_TRIAD(:,i), P_Triad(i).a ] = TRIAD(r,b,sigma,sgn);
    Euler_ang_TRIAD(:,i) = qtoEuler(q_TRIAD(:,i));
 
    [q_qMethod(:,i), P_qMethod(i).a] = qMethod(r,b,sigma,sgn);
    Euler_ang_qMethod(:,i) = qtoEuler(q_qMethod(:,i));
end

%% Only Sun and Nadir Vectors Obs.
str = 'Sun-Nadir'
for i = 1:N+1
    
    sigma = [std(S_b(:,i)) + std(S_o(:,i));
        std(N_b(:,i)) + std(N_o(:,i))];
    sgn = sign(q_true(:,i));
    
    b = [S_b(:,i), N_b(:,i)];
    r = [S_o(:,i), N_o(:,i)];
    
    [q_TRIAD(:,i), P_Triad(i).a, A_TRIAD(i).a] = TRIAD(r,b,sigma,sgn);
    Euler_ang_TRIAD(:,i) = qtoEuler(q_TRIAD(:,i));
    
    [q_qMethod(:,i), P_qMethod(i).a] = qMethod(r,b,sigma,sgn);
    Euler_ang_qMethod(:,i) = qtoEuler(q_qMethod(:,i));
end

%% q Method All Combinations 
str = 'Sun-Magnetometer-Nadir'
for i = 1:N+1
    
    sigma = [std(S_b(:,i)) + std(S_o(:,i));
        std(N_b(:,i)) + std(N_o(:,i));
        std(H_b(:,i)) + std(H_o(:,i))];
       
    sgn = sign(q_true(:,i));
    
    b = [S_b(:,i), H_b(:,i), N_b(:,i)];
    r = [S_o(:,i), H_o(:,i), N_o(:,i)];
 
    [q_qMethod3(:,i)] = qMethod3(r,b,sigma,sgn);
    Euler_ang_qMethod3(:,i) = qtoEuler(q_qMethod3(:,i));
end

RMSE_q_Method3 = sqrt( sum( abs( rad2deg(Euler_ang_true)' - rad2deg(Euler_ang_qMethod3)' ).^2 ) / (length(Euler_ang_true)) )

%% Single Frame Aided EKF
tic
[X,P_EKF] = EKF_plus(q_TRIAD,Euler_ang_TRIAD,w_angular,P_Triad,N);
[X2,P_q] = EKF_plus(q_qMethod,Euler_ang_qMethod,w_angular,P_qMethod,N);
n = length(X);
for i = 1:n
    Euler_ang_EKF(:,i) = qtoEuler(X(1:4,i));
    Euler_ang_EKF2(:,i) = qtoEuler(X2(1:4,i));
end
% Root Mean Square Error (RMSE)
toc
RMSE_TRIAD = sqrt( sum( abs( rad2deg(Euler_ang_true)' - rad2deg(Euler_ang_TRIAD)' ).^2 ) / n )

RMSE_q_Method = sqrt( sum( abs( rad2deg(Euler_ang_true)' - rad2deg(Euler_ang_qMethod)' ).^2 ) / n )

RMSE_TRIAD_EKF = sqrt( sum( abs( rad2deg(Euler_ang_true)' - rad2deg(Euler_ang_EKF)' ).^2 ) / n )

RMSE_qMethod_EKF2 = sqrt( sum( abs( rad2deg(Euler_ang_true)' - rad2deg(Euler_ang_EKF2)' ).^2 ) / n )


 %%
 PLOTTING(n,time,str,rad2deg(Euler_ang_EKF),...
    rad2deg(Euler_ang_TRIAD),...
    rad2deg(Euler_ang_qMethod),...
    rad2deg(Euler_ang_true))
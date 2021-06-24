% Extended Kalman filter example
clc;
clear all;
%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end
close all;

% Discrete time step
dt = 0.1;
u=[-1.5 2 1];

% Initial State
x0 = [0 0 0]';

% Prior
mu = [0 0 0]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)

% Process noise covariance matrix
R=[0.01 0 0;
    0 0.01 0;
    0 0 0.1*(pi/180)].^2;
[RE, Re] = eig (R);

% Measurement noise covariance matrix
Q=[0.5 0 0;
    0 0.5 0;
    0 0 10*(pi/180)].^2;
[QE, Qe] = eig(Q);

% Simulation Initializations
Tf = 15;
T = 0:dt:Tf;
n = length(mu);
x = zeros(n,length(T));
x_ideal = zeros(n,length(T)); %
x(:,1) = x0;
x_ideal(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Prediction update
    x_ideal(:,t) = omnibot_motion_model(x_ideal(:,t-1),u,dt);
    x(:,t) = omnibot_motion_model(x(:,t-1),u,dt) + e;
    g = x_ideal(:,t);
    Gt = omnibot_linearize_motion_model(x(:,t),u);
    Ht = omnibot_linearize_measurement_model(x(:,t));

    % Measurement update
    % Select a motion disturbance
    d = sqrt(Q)*randn(m,1);
    % Determine measurement
    y(:,t) = omnibot_sensor_model(x_ideal(:,t)) + d;
    Y = y(:,t);

    
    %% Extended Kalman Filter Estimation
    [mu,S,K,mup] = EKF(g,Gt,Ht,S,Y,@omnibot_sensor_model,R,Q);
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    
    %% Plot results
    figure(1);clf; hold on;
    plot(x(1,2:t),x(2,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    plot(y(1,2:t),y(2,2:t), 'gx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    xlabel('x')
    ylabel('y');
    
    %for printing title and legend in video
    text(-0.5,1.3,'Omnibot Simulation');
    text(2.25,1.3,'True State','Color','red');  %printing text is faster than printing legend
    text(2.25,1.1,'EKF Estimate','Color','blue');
    text(2.25,0.9,'Measurement','Color','green');
    
    %for printing axis in video
    for i=-8:2:8
        text(i-0.2,-13.5,num2str(i));
    end
    for j=-14:2:2
        text(-7.7,j,num2str(j));
    end
    axis equal
    axis([-2.5 3.5 -3.5 1.5])
    pause(0.0001);
    
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    
end

if (makemovie) close(vidObj); end

t = T;
state_x_true = x(1,:);
state_x_est = mu_S(1,:);
state_x_meas = y(1,:);

state_y_true = x(2,:);
state_y_est = mu_S(2,:);
state_y_meas = y(2,:);

state_theta_true = x(3,:);
state_theta_est = mu_S(3,:);
state_theta_meas = y(3,:);

figure(2); hold on;
plot(t, state_x_true, 'ro--');
plot(t, state_x_est, 'bx--'); 
plot(t, state_x_meas, 'gx--');
title('Omnibot Simulation: X-Position');
ylabel('x');
xlabel('t'),
legend('true state', 'estimate', 'measurement');

figure(3); hold on;
plot(t, state_y_true, 'ro--');
plot(t, state_y_est, 'bx--'); 
plot(t, state_y_meas, 'gx--');
title('Omnibot Simulation: Y-Position');
ylabel('y');
xlabel('t'),
legend('true state', 'estimate', 'measurement');

figure(4); hold on;
plot(t, state_theta_true, 'ro--');
plot(t, state_theta_est, 'bx--'); 
plot(t, state_theta_meas, 'gx--');
title('Omnibot Simulation: Theta-Position');
ylabel('theta');
xlabel('t'),
legend('true state', 'estimate', 'measurement');
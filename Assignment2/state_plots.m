clc; clear all; close all;

load T
load x
load mu_S
load mup
load y

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

figure(1); hold on;
plot(t, state_x_true, 'ro--');
plot(t, state_x_est, 'bx--'); 
plot(t, state_x_meas, 'gx--');
title('Omnibot Simulation: X-Position');
ylabel('x');
xlabel('t'),

figure(2); hold on;
plot(t, state_y_true, 'ro--');
plot(t, state_y_est, 'bx--'); 
plot(t, state_y_meas, 'gx--');
title('Omnibot Simulation: Y-Position');
ylabel('y');
xlabel('t'),

figure(3); hold on;
plot(t, state_theta_true, 'ro--');
plot(t, state_theta_est, 'bx--'); 
plot(t, state_theta_meas, 'gx--');
title('Omnibot Simulation: Theta-Position');
ylabel('theta');
xlabel('t'),

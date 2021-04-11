% main script
%
%% environment setup
clear;  close all; clc;
 
%% helper functions
addpath(genpath('./helper_functions'));
addpath('./models');

%% script parameters


%%
trajectory = @circular_traj;

model = Quadrotor('traj',trajectory);
model.controller = @model.ctrlGeoPD;


%%  simulate
% time
t0 = 0;
tf = 20;

% initial condition
xQ0 = 0.1*[1;1;1];
vQ0 = zeros(3,1);
R0 = RPY2Rot_ZXY([178*pi/180,0,0]);
Om0 = zeros(3,1);

q0 = model.zipState(xQ0,vQ0,R0,Om0);

sol = model.simulate([t0, tf], q0, @ode45);

% results
results.t = sol.x;
results.x = sol.y;
results.td = sol.x;
results.xd = model.getRefState(sol.x);
 
%% animate
opts.data = results;
model.animate(opts);

%% plots
figure; 
subplot(3,1,1);
hold on;
plot(sol.x,results.x(1,:),'r','linewidth',1);
plot(sol.x,results.xd(1,:),'b','linewidth',1);
grid on; grid minor; legend('x','x_d');
subplot(3,1,2);
hold on;
plot(sol.x,results.x(2,:),'r','linewidth',1);
plot(sol.x,results.xd(2,:),'b','linewidth',1);
grid on; grid minor;  legend('y','y_d');
subplot(3,1,3);
hold on;
plot(sol.x,results.x(3,:),'r','linewidth',1);
plot(sol.x,results.xd(3,:),'b','linewidth',1);
grid on; grid minor;  legend('z','z_d');









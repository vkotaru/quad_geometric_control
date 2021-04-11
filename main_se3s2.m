% main script


%% environment setup
clear;  close all; clc;
 
%% helper functions
addpath(genpath('./helper_functions'));
addpath('./models');

%% script parameters


%%
trajectory = @circular_traj;

model = Quadrotorload('traj',trajectory);
model.controller = @model.ctrlGeoPD;


%%  
% time
t0 = 0;
tf = 20;

% initial condition
r0 = 0;
p0 = 0;
y0 = 0;

xL0 = 0.*[1;1;1];
vL0 = zeros(3,1);
R0 = RPY2Rot_ZXY([r0*pi/180,p0*pi/180,y0*pi/180]);
Om0 = zeros(3,1);
q0 = [0;0;-1];
om0 = zeros(3,1);

xq0 = model.zipState(xL0,vL0,R0,Om0,q0,om0);

%% simulate
sol = model.simulate([t0, tf], xq0, @ode45);

% results
results.t = sol.x;
results.x = sol.y;
results.td = sol.x;
results.xd = model.getRefState(sol.x);
 
%% animate
opts.data = results;
opts.RATE = 12.5;
model.animate(opts);

%% plots
figure; 
subplot(3,1,1);
hold on;
plot(sol.x,results.x(1,:),'r','linewidth',1);
plot(sol.x,results.xd(1,:),'b','linewidth',1);
grid on; grid minor; latex_legend({'$$x$$','$$x_d$$'});
subplot(3,1,2);
hold on;
plot(sol.x,results.x(2,:),'r','linewidth',1);
plot(sol.x,results.xd(2,:),'b','linewidth',1);
grid on; grid minor; latex_legend({'$$y$$','$$y_d$$'});
subplot(3,1,3);
hold on;
plot(sol.x,results.x(3,:),'r','linewidth',1);
plot(sol.x,results.xd(3,:),'b','linewidth',1);
grid on; grid minor; latex_legend({'$$z$$','$$z_d$$'});









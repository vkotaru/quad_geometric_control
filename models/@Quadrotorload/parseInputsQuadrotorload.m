function obj = parseInputsQuadrotorload(obj,varargin)
%% function to parse inputs for Quadrotor model

% default parameters
default_mQ = 0.85;
default_mL = 0.5;
default_l = 1;
default_J = diag([0.557, 0.557, 1.05]*10e-2);
default_traj = @(t) struct('x',zeros(3,1));

%%
p = inputParser;
ishandle = @(x) isa(x,'function_handle');

addOptional(p,'l',default_l);
addOptional(p,'J',default_J);
addOptional(p,'mQ',default_mQ);
addOptional(p,'mL',default_mL);
addOptional(p,'traj',default_traj,ishandle);

parse(p,varargin{:});

%%
obj.mQ = p.Results.mQ;
obj.mL = p.Results.mL;
obj.J = p.Results.J;
obj.Jinv= inv(obj.J);
obj.l = p.Results.l;

% params = [mQ; mL; l; g; reshape(J,9,1); reshape(Jinv,9,1)];
obj.params = [obj.mQ; obj.mL; obj.l; obj.g; reshape(obj.J,9,1); reshape(obj.Jinv,9,1)];

obj.getFlatTraj = p.Results.traj;
%% ctrl parameters
obj.ctrlParams = struct('kx', 2*diag([0.4687,0.4687,0.4687]),...
                            'kv', 2*diag([.75, .75,0.75]),...
                            'kR', diag([400.0, 400.0,400.0]),...
                            'kOm', 0.5*diag([80.0, 80.0,80.0]),...
                            'kq', -diag([6,6,6]),...
                            'kom', -diag([1.6, 1.6, 1.6]));

% epsilon = 0.1 ;
%% 

obj.bounds.x.lb = [];
obj.bounds.x.ub = [];

obj.bounds.u.ub = [100; 10; 10; 10];
obj.bounds.u.lb = [0; -10; -10; -10];

end
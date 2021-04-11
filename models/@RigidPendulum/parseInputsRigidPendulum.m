function obj = parseInputsQuadrotor(obj,varargin)
%% function to parse inputs for Quadrotor model

% default parameters
default_m = 0.85;
default_l = 1;
default_J = diag([0.557, 0.557, 1.05]*10e-2);
default_traj = @(t) struct('x',zeros(3,1));
default_r = [0.0; 0.0; -1.0];

%%
p = inputParser;
ishandle = @(x) isa(x,'function_handle');

addOptional(p,'l',default_l);
addOptional(p,'J',default_J);
addOptional(p,'mass',default_m);
addOptional(p,'r',default_r);
addOptional(p,'traj',default_traj,ishandle);


parse(p,varargin{:});

%%
obj.m = p.Results.mass;
obj.J = p.Results.J;
obj.Jinv= inv(obj.J);
obj.l = p.Results.l;
obj.r = p.Results.r;

obj.params = [obj.m; obj.g; reshape(obj.J,9,1); reshape(obj.Jinv,9,1); obj.r];

obj.getFlatTraj = p.Results.traj;
%% ctrl parameters
obj.ctrlParams = struct('kx', 2*diag([1.0, 1.0,1.0]),...
                            'kv', 2*diag([.8, .8,0.8]),...
                            'kR', 0.5*diag([400.0, 400.0,400.0]),...
                            'kOm', 0.5*diag([80.0, 80.0,80.0]));

% epsilon = 0.1 ;
%% 

obj.bounds.x.lb = [];
obj.bounds.x.ub = [];

obj.bounds.u.ub = [100; 10; 10; 10];
obj.bounds.u.lb = [0; -10; -10; -10];

end
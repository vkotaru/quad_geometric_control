classdef RigidPendulum
%fdasf
properties
    m = 0.85
    J  = diag([0.557, 0.557, 1.05]*10e-2)
    Jinv = inv(diag([0.557, 0.557, 1.05]*10e-2))
    l = 1
    r = [0.0; 0.0; -1.0];

    g = 9.81
    e1 = [1;0;0];
    e2 = [0;1;0];
    e3 = [0;0;1];
    dim = 3

    type
    params
    
    bounds
    getFlatTraj@function_handle
    controller@function_handle
    calcCtrlInput@function_handle
    ctrlParams@struct
end

properties (Constant = true)
    nx = 12;
    nu = 3;
end

methods
	% class constructor
    function obj = RigidPendulum(varargin)
        fprintf("Creating an instance of RigidPendulum\n");
        obj = obj.parseInputsRigidPendulum(varargin{:});
        obj.type = 'rigidpendulumonSO3';                
    end
    
    % set property values
    function obj = setProperty(obj, propertyName, value)
        if isprop(obj, propertyName)
            set(obj, propertyName, value);
        else
            error([propertyName ' not a property of class ',class(obj)]);
        end
    end
%%  dynamics  
    function [R,Om] = unzipState(obj,x)
        R = reshape(x(1:9),3,3);
        Om = x(10:12);
    end
    
    function [x] = zipState(obj,R,Om)
       x = [reshape(R,9,1); Om]; 
    end
    
    function sol = simulate(obj, tspan, x0, solver,varargin)
        if nargin > 4
            obj.calcCtrlInput = @(t,x) varargin{1};
        else
            obj.calcCtrlInput = obj.controller;
        end
        
%         keyboard;
        odefun = @(t,x)systemDynamics(obj, t, x);
        sol = solver(odefun, tspan, x0);
    end

    function ddx = systemDynamics(obj, t, x)
        u = obj.calcCtrlInput(t,x);
        [fvec, gvec] = obj.getVecFields(x,obj.params);
        ddx = fvec + gvec*u; 
    end
    
    function[qref] = getRefState(obj,t)
        qref = zeros(obj.nx,length(t));
       for i = 1:length(t)
          q_ = obj.flat2state(obj.getFlatTraj(t(i)));
          qref(:,i) = obj.zipState(q_.R,q_.Omega);
       end
        
    end
    
 
%%  methods defined externally
    % dynamics
    % [Dynamics,fVec,gVec] = getVecFields(obj,x,params);
    % [A,B] = getLinearDynamics(obj,xref,uref,params);
    
    % differential flatness
    % [flats] = getFlatTraj(obj,t);
    [ref] = flat2state(obj,flats);
    
    % animation
    animate(obj,opts_in);
    
    % controls
    [u] = ctrlGeoPD(obj,t,x);
    
end

end
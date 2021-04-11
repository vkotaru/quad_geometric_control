classdef Quadrotorload
% Quadrotorload

properties
    mQ = 0.85
    mL = 0.5
    J  = diag([0.557, 0.557, 1.05]*10e-2)
    Jinv = inv(diag([0.557, 0.557, 1.05]*10e-2))
    l = 1

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
    nx = 24;
    nu = 4;
end

methods
	% class constructor
    function obj = Quadrotorload(varargin)
        obj = obj.parseInputsQuadrotorload(varargin{:});
        obj.type = 'quadrotorloadonSE3xS2';                
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
    function [xL,vL,R,Om,q,om] = unzipState(obj,x)
        xL = x(1:3);
        vL = x(4:6);
        R = reshape(x(7:15),3,3);
        Om = x(16:18);
        q = x(19:21);
        om = x(22:24);
    end
    
    function [x] = zipState(obj,xL,vL,R,Om,q,om)
       x = [xL; vL; reshape(R,9,1); Om; q; om]; 
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
          x_ = obj.flat2state(obj.getFlatTraj(t(i)));
          qref(:,i) = obj.zipState(x_.xL,x_.vL,x_.R,x_.Omega,x_.q,cross2(x_.q,x_.dq));
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
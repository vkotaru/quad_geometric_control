function[u] = ctrlGeoPD(obj,t,x)
% function to calculate non-linear geometric pd input

% keyboard;
%%
% extracting states
[R,Omega] = obj.unzipState(x);

% calculating ref traj
ref = obj.flat2state(obj.getFlatTraj(t));

%% attitude ctrl
Rd = ref.R;
Omegad = ref.Omega;
dOmegad = ref.dOmega;

% calculating errors
err_R = 1/2 * vee_map(Rd'*R - R'*Rd) ;
err_Om = Omega - R'*Rd*Omegad ;

% inputs
M = -obj.ctrlParams.kR*err_R - obj.ctrlParams.kOm*err_Om + vec_cross(Omega, obj.J*Omega)...
    - obj.J*(hat_map(Omega)*R'*Rd*Omegad - R'*Rd*dOmegad) ;

%% output
u = [M];

end
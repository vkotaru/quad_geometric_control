function[u] = ctrlGeoPD(obj,t,x)
% function to calculate non-linear geometric pd input

% keyboard;
%%
% extracting states
[xQ,vQ,R,Omega] = obj.unzipState(x);

% calculating ref traj
ref = obj.flat2state(obj.getFlatTraj(t));

%% position ctrl

% calculating errors
err_x = xQ-ref.x;
err_v = vQ-ref.v;

Fff = obj.m*(obj.g*obj.e3 + ref.a);
Fpd = -obj.ctrlParams.kx*err_x - obj.ctrlParams.kv*err_v;
F = Fff+Fpd;

%% attitude ctrl

% calculating desired orientation
b3c = F/norm(F);
b3 = R(:,3) ;

b1d = obj.e1;
b1c = -cross2(b3c,cross2(b3c,b1d));
b1c = b1c/norm(cross2(b3c,b1d));
Rc = [b1c cross2(b3c,b1c) b3c];
Rd = Rc;

Omegad = ref.Omega;
dOmegad = ref.dOmega;


% calculating errors
err_R = 1/2 * vee(Rd'*R - R'*Rd) ;
err_Om = Omega - R'*Rd*Omegad ;

% inputs
f = dot2(F,b3) ;
M = -obj.ctrlParams.kR*err_R - obj.ctrlParams.kOm*err_Om + cross2(Omega, obj.J*Omega)...
    - obj.J*(hat(Omega)*R'*Rd*Omegad - R'*Rd*dOmegad) ;



%% output
u = [f;M];

end
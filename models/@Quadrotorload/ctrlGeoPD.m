function[u] = ctrlGeoPD(obj,t,x)
% function to calculate non-linear geometric pd input

% keyboard;
%%
% extracting states
[xL,vL,R,Omega,q,om] = obj.unzipState(x);
dq = cross(om,q);

% calculating ref traj
ref = obj.flat2state(obj.getFlatTraj(t));

%% load-position ctrl

% calculating errors
err_xL = xL-ref.xL;
err_vL = vL-ref.vL;

Fff = (obj.mQ+obj.mL)*(ref.aL+obj.g*obj.e3) + obj.mQ*obj.l*dot2(dq,dq)*q;
Fpd = -obj.ctrlParams.kx*err_xL - obj.ctrlParams.kv*err_vL;
A = Fff+Fpd;

% desired load attitude
qc = -A/norm(A);

%% load-attitude ctrl
qd = qc; % ref.q;
dqd = ref.dq;
d2qd = ref.d2q;

% calculating errors
err_q = hat(q)^2*qd;
err_dq = dq - cross2(cross2(qd, dqd), q);

Fpd =  -obj.ctrlParams.kq*err_q-obj.ctrlParams.kom*err_dq;
Fff = (obj.mQ*obj.l)*dot2(q, cross2(qd,dqd))*cross2(q,dq)+...
            (obj.mQ*obj.l)*cross2(cross2(qd, d2qd), q);
Fn = dot2(A,q)*q;
F = Fpd-Fff+Fn;

%% attitude ctrl

% calculating desired orientation
b3c = F/norm(F);
b3 = R(:,3) ;

b1d = obj.e1;
b1c = -cross2(b3c,cross2(b3c,b1d));
b1c = b1c/norm(cross2(b3c,b1d));
Rc = [b1c cross2(b3c,b1c) b3c];
Rd = Rc; % ref.R;

Omegad = ref.Omega;
dOmegad = ref.dOmega;


% calculating errors
err_R = 1/2 * vee(Rd'*R - R'*Rd) ;
err_Om = Omega - R'*Rd*Omegad ;

%% inputs
f = dot2(F,b3) ;
M = -obj.ctrlParams.kR*err_R - obj.ctrlParams.kOm*err_Om + cross2(Omega, obj.J*Omega)...
    - obj.J*(hat(Omega)*R'*Rd*Omegad - R'*Rd*dOmegad) ;

%% output
u = [f;M];

end
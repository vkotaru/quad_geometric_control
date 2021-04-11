function [ref] = flat2state(obj,flats)
%% 
% Function to calculate the feedforward reference states and inputs for a
% given position trajectory using differential flatness

ref.xL = flats.x;
ref.vL = flats.dx;
ref.aL = flats.d2x;
ref.daL = flats.d3x;
ref.d2aL = flats.d4x;
ref.d3aL = flats.d5x;
ref.d4aL = flats.d6x;

b1d = obj.e1;
db1d = zeros(3,1);
d2b1d = zeros(3,1);

[ref.q,ref.dq,ref.d2q,ref.d3q,ref.d4q] = getqVectors(obj.mL,obj.g,ref.aL,ref.daL,ref.d2aL,ref.d3aL,ref.d4aL);

% % >>> q
% Tp = -obj.mL*(ref.aL + obj.g*obj.e3);
% norm_Tp = norm(Tp) ;
% ref.q = Tp / norm_Tp ;
% % >>> dq
% dTp = -obj.mL*ref.daL;
% dnorm_Tp = 1/norm_Tp * vec_dot(Tp, dTp) ;
% ref.dq = (dTp - ref.q*dnorm_Tp) / norm_Tp ;
% % >>> d2q    
% d2Tp = -obj.mL*ref.d2aL ;
% d2norm_Tp = ( vec_dot(dTp, dTp) + vec_dot(Tp, d2Tp) - dnorm_Tp^2 ) / norm_Tp ;
% ref.d2q = ( d2Tp - ref.dq*dnorm_Tp - ref.q*d2norm_Tp - ref.dq*dnorm_Tp) / norm_Tp ;
% % >>> d3q    
% d3Tp = -obj.mL*ref.d3aL ;
% d3norm_Tp = ( 2*vec_dot(d2Tp, dTp) + vec_dot(dTp, d2Tp)+vec_dot(Tp, d3Tp) - 3*dnorm_Tp*d2norm_Tp) / norm_Tp ;
% ref.d3q = (d3Tp - ref.d2q*dnorm_Tp-ref.dq*d2norm_Tp - ref.dq*d2norm_Tp-ref.q*d3norm_Tp - ref.d2q*dnorm_Tp-ref.dq*d2norm_Tp - ref.d2q*dnorm_Tp) / norm_Tp ;
% % >>> d4q
% d4Tp = -obj.mL*ref.d4aL ;
% d4norm_Tp = ( 2*vec_dot(d3Tp, dTp)+2*vec_dot(d2Tp, d2Tp) + vec_dot(d2Tp, d2Tp)+vec_dot(dTp, d3Tp) + vec_dot(dTp, d3Tp)+vec_dot(Tp, d4Tp) - 3*d2norm_Tp^2-3*dnorm_Tp*d3norm_Tp ...
%     - d3norm_Tp*dnorm_Tp) / norm_Tp ;
% ref.d4q = ( d4Tp - ref.d3q*dnorm_Tp-ref.d2q*d2norm_Tp - ref.d2q*d2norm_Tp-ref.dq*d3norm_Tp - ref.d2q*d2norm_Tp-ref.dq*d3norm_Tp - ref.dq*d3norm_Tp-ref.q*d4norm_Tp ...
%     - ref.d3q*dnorm_Tp-ref.d2q*d2norm_Tp - ref.d2q*d2norm_Tp-ref.dq*d3norm_Tp - ref.d3q*dnorm_Tp-ref.d2q*d2norm_Tp - ref.d3q*dnorm_Tp ) / norm_Tp ;

ref.xQ = ref.xL - obj.l*ref.q ;
ref.vQ = ref.vL - obj.l*ref.dq ;
ref.aQ = ref.aL - obj.l*ref.d2q ;
ref.daQ = ref.daL - obj.l*ref.d3q ;
ref.d2aQ = ref.d2aL - obj.l*ref.d4q ;

[R,Omega,dOmega,F,M] = getForceNMoment(obj.mQ, obj.J, obj.g, ref.aQ, ref.daQ, ref.d2aQ, b1d, db1d, d2b1d );

ref.R = R;
ref.Omega = Omega;
ref.dOmega = dOmega;
ref.F =F;
ref.M = M;
ref.f = dot2(F,R*obj.e3);

end

function [q,dq,d2q,d3q,d4q] = getqVectors(mL,g,aL,daL,d2aL,d3aL,d4aL)
e3 = [0; 0; 1];
    % >>> q
    Tp = -mL*(aL + g*e3);
    norm_Tp = norm(Tp) ;
    q = Tp / norm_Tp ;
    % >>> dq
    dTp = -mL*daL;
    dnorm_Tp = 1/norm_Tp * dot2(Tp, dTp) ;
    dq = (dTp - q*dnorm_Tp) / norm_Tp ;
    % >>> d2q    
    d2Tp = -mL*d2aL ;
    d2norm_Tp = ( dot2(dTp, dTp) + dot2(Tp, d2Tp) - dnorm_Tp^2 ) / norm_Tp ;
    d2q = ( d2Tp - dq*dnorm_Tp - q*d2norm_Tp - dq*dnorm_Tp) / norm_Tp ;
    % >>> d3q    
    d3Tp = -mL*d3aL ;
    d3norm_Tp = ( 2*dot2(d2Tp, dTp) + dot2(dTp, d2Tp)+dot2(Tp, d3Tp) - 3*dnorm_Tp*d2norm_Tp) / norm_Tp ;
    d3q = (d3Tp - d2q*dnorm_Tp-dq*d2norm_Tp - dq*d2norm_Tp-q*d3norm_Tp - d2q*dnorm_Tp-dq*d2norm_Tp - d2q*dnorm_Tp) / norm_Tp ;
    % >>> d4q
    d4Tp = -mL*d4aL ;
    d4norm_Tp = ( 2*dot2(d3Tp, dTp)+2*dot2(d2Tp, d2Tp) + dot2(d2Tp, d2Tp)+dot2(dTp, d3Tp) + dot2(dTp, d3Tp)+dot2(Tp, d4Tp) - 3*d2norm_Tp^2-3*dnorm_Tp*d3norm_Tp ...
        - d3norm_Tp*dnorm_Tp) / norm_Tp ;
    d4q = ( d4Tp - d3q*dnorm_Tp-d2q*d2norm_Tp - d2q*d2norm_Tp-dq*d3norm_Tp - d2q*d2norm_Tp-dq*d3norm_Tp - dq*d3norm_Tp-q*d4norm_Tp ...
        - d3q*dnorm_Tp-d2q*d2norm_Tp - d2q*d2norm_Tp-dq*d3norm_Tp - d3q*dnorm_Tp-d2q*d2norm_Tp - d3q*dnorm_Tp ) / norm_Tp ;
end


function [R,Omega,dOmega,F,M] = getForceNMoment(mQ,J,g,axQ, daxQ, d2axQ,b1d,db1d, d2b1d )
        
    e1 = [1;0;0];
    e2 = [0; 1; 0];
    e3 = [0; 0; 1];
    
    fb3 = mQ*(axQ+g*e3);
    norm_fb3 = norm(fb3) ;
    f = norm_fb3 ;
    b3 = fb3 / norm_fb3 ;
    b3_b1d = cross2(b3, b1d) ;
    norm_b3_b1d = norm(b3_b1d) ;
    b1 = - cross2(b3, b3_b1d) / norm_b3_b1d ;
    b2 = cross2(b3, b1) ;
    R = [b1 b2 b3] ;
    
    dfb3 = mQ*(daxQ);
    dnorm_fb3 = dot2(fb3, dfb3) / norm_fb3 ;
    db3 = (dfb3*norm_fb3 - fb3*dnorm_fb3) / norm_fb3^2 ;
    db3_b1d = cross2(db3, b1d) + cross2(b3, db1d) ;
    dnorm_b3_b1d = dot2(b3_b1d, db3_b1d) / norm_b3_b1d ;
    db1 = (-cross2(db3,b3_b1d)-cross2(b3,db3_b1d) - b1*dnorm_b3_b1d) / norm_b3_b1d ;
    db2 = cross2(db3, b1) + cross2(b3, db1) ;
    dR = [db1 db2 db3] ;
    Omega = vee(R'*dR) ;
    
    d2fb3 = mQ*(d2axQ);
    d2norm_fb3 = (dot2(dfb3, dfb3)+dot2(fb3, d2fb3) - dnorm_fb3*dnorm_fb3) / norm_fb3 ;
    d2b3 = ( (d2fb3*norm_fb3+dfb3*dnorm_fb3 - dfb3*dnorm_fb3-fb3*d2norm_fb3)*norm_fb3^2 - db3*norm_fb3^2*2*norm_fb3*dnorm_fb3 ) / norm_fb3^4 ;
    d2b3_b1d = cross2(d2b3, b1d)+cross2(db3, db1d) + cross2(db3, db1d)+cross2(b3, d2b1d) ;
    d2norm_b3_b1d = ( (dot2(db3_b1d,db3_b1d)+dot2(b3_b1d,d2b3_b1d))*norm_b3_b1d - dot2(b3_b1d, db3_b1d)*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
    d2b1 = ( (-cross2(d2b3,b3_b1d)-cross2(db3,db3_b1d) - cross2(db3,db3_b1d)-cross2(b3,d2b3_b1d) - db1*dnorm_b3_b1d-b1*d2norm_b3_b1d )*norm_b3_b1d - db1*norm_b3_b1d*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
    d2b2 = cross2(d2b3, b1)+cross2(db3, db1) + cross2(db3, db1)+cross2(b3, d2b1) ;
    d2R = [d2b1 d2b2 d2b3] ;
    dOmega = vee( dR'*dR + R'*d2R ) ; %vee( dR'*dR + R'*d2R, true ) ;
    
M = J*dOmega + cross2(Omega, J*Omega) ;
F = fb3;

end


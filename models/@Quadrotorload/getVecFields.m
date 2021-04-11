function [fVec,gVec] = getVecFields(obj,in2,in3)
%GETVECFIELDS
%    [FVEC,GVEC] = GETVECFIELDS(OBJ,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    08-Jan-2019 15:08:05

J1_1 = in3(5,:);
J1_2 = in3(8,:);
J1_3 = in3(11,:);
J2_1 = in3(6,:);
J2_2 = in3(9,:);
J2_3 = in3(12,:);
J3_1 = in3(7,:);
J3_2 = in3(10,:);
J3_3 = in3(13,:);
Jinv1_1 = in3(14,:);
Jinv1_2 = in3(17,:);
Jinv1_3 = in3(20,:);
Jinv2_1 = in3(15,:);
Jinv2_2 = in3(18,:);
Jinv2_3 = in3(21,:);
Jinv3_1 = in3(16,:);
Jinv3_2 = in3(19,:);
Jinv3_3 = in3(22,:);
Om1 = in2(16,:);
Om2 = in2(17,:);
Om3 = in2(18,:);
R1_1 = in2(7,:);
R1_2 = in2(10,:);
R1_3 = in2(13,:);
R2_1 = in2(8,:);
R2_2 = in2(11,:);
R2_3 = in2(14,:);
R3_1 = in2(9,:);
R3_2 = in2(12,:);
R3_3 = in2(15,:);
g = in3(4,:);
l = in3(3,:);
mL = in3(2,:);
mQ = in3(1,:);
om1 = in2(22,:);
om2 = in2(23,:);
om3 = in2(24,:);
q1 = in2(19,:);
q2 = in2(20,:);
q3 = in2(21,:);
v1 = in2(4,:);
v2 = in2(5,:);
v3 = in2(6,:);
t7 = om1.*q2;
t8 = om2.*q1;
t2 = t7-t8;
t10 = om1.*q3;
t11 = om3.*q1;
t3 = t10-t11;
t13 = om2.*q3;
t14 = om3.*q2;
t4 = t13-t14;
t5 = mL+mQ;
t6 = 1.0./t5;
t9 = t2.^2;
t12 = t3.^2;
t15 = t4.^2;
t16 = t9+t12+t15;
t17 = J1_1.*Om1;
t18 = J1_2.*Om2;
t19 = J1_3.*Om3;
t20 = t17+t18+t19;
t21 = J2_1.*Om1;
t22 = J2_2.*Om2;
t23 = J2_3.*Om3;
t24 = t21+t22+t23;
t25 = J3_1.*Om1;
t26 = J3_2.*Om2;
t27 = J3_3.*Om3;
t28 = t25+t26+t27;
t29 = Om2.*t20;
t35 = Om1.*t24;
t30 = t29-t35;
t31 = Om3.*t20;
t36 = Om1.*t28;
t32 = t31-t36;
t33 = Om3.*t24;
t37 = Om2.*t28;
t34 = t33-t37;
fVec = [v1;v2;v3;-l.*mQ.*q1.*t6.*t16;-l.*mQ.*q2.*t6.*t16;-g-l.*mQ.*q3.*t6.*t16;-Om2.*R1_3+Om3.*R1_2;-Om2.*R2_3+Om3.*R2_2;-Om2.*R3_3+Om3.*R3_2;Om1.*R1_3-Om3.*R1_1;Om1.*R2_3-Om3.*R2_1;Om1.*R3_3-Om3.*R3_1;-Om1.*R1_2+Om2.*R1_1;-Om1.*R2_2+Om2.*R2_1;-Om1.*R3_2+Om2.*R3_1;Jinv1_3.*t30-Jinv1_2.*t32+Jinv1_1.*t34;Jinv2_3.*t30-Jinv2_2.*t32+Jinv2_1.*t34;Jinv3_3.*t30-Jinv3_2.*t32+Jinv3_1.*t34;t4;-t10+t11;t2;0.0;0.0;0.0];
if nargout > 1
    t38 = R1_3.*q1;
    t39 = R2_3.*q2;
    t40 = R3_3.*q3;
    t41 = t38+t39+t40;
    t42 = 1.0./l;
    t43 = 1.0./mQ;
    gVec = reshape([0.0,0.0,0.0,q1.*t6.*t41,q2.*t6.*t41,q3.*t6.*t41,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t42.*t43.*(R2_3.*q3-R3_3.*q2),-t42.*t43.*(R1_3.*q3-R3_3.*q1),t42.*t43.*(R1_3.*q2-R2_3.*q1),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_1,Jinv2_1,Jinv3_1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_2,Jinv2_2,Jinv3_2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_3,Jinv2_3,Jinv3_3,0.0,0.0,0.0,0.0,0.0,0.0],[24,4]);
end

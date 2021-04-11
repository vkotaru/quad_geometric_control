function [fVec,gVec] = getVecFields(obj,in2,in3)
%GETVECFIELDS
%    [FVEC,GVEC] = GETVECFIELDS(OBJ,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    05-Jan-2019 17:47:13

J1_1 = in3(3,:);
J1_2 = in3(6,:);
J1_3 = in3(9,:);
J2_1 = in3(4,:);
J2_2 = in3(7,:);
J2_3 = in3(10,:);
J3_1 = in3(5,:);
J3_2 = in3(8,:);
J3_3 = in3(11,:);
Jinv1_1 = in3(12,:);
Jinv1_2 = in3(15,:);
Jinv1_3 = in3(18,:);
Jinv2_1 = in3(13,:);
Jinv2_2 = in3(16,:);
Jinv2_3 = in3(19,:);
Jinv3_1 = in3(14,:);
Jinv3_2 = in3(17,:);
Jinv3_3 = in3(20,:);
Om1 = in2(10,:);
Om2 = in2(11,:);
Om3 = in2(12,:);
R1_1 = in2(1,:);
R1_2 = in2(4,:);
R1_3 = in2(7,:);
R2_1 = in2(2,:);
R2_2 = in2(5,:);
R2_3 = in2(8,:);
R3_1 = in2(3,:);
R3_2 = in2(6,:);
R3_3 = in2(9,:);
g = in3(2,:);
m = in3(1,:);
r1 = in3(21,:);
r2 = in3(22,:);
r3 = in3(23,:);
t2 = J1_1.*Om1;
t3 = J1_2.*Om2;
t4 = J1_3.*Om3;
t5 = t2+t3+t4;
t6 = J2_1.*Om1;
t7 = J2_2.*Om2;
t8 = J2_3.*Om3;
t9 = t6+t7+t8;
t10 = J3_1.*Om1;
t11 = J3_2.*Om2;
t12 = J3_3.*Om3;
t13 = t10+t11+t12;
t14 = Om2.*t5;
t15 = R3_1.*r2;
t30 = R3_2.*r1;
t16 = t15-t30;
t17 = g.*m.*t16;
t29 = Om1.*t9;
t18 = t14+t17-t29;
t19 = Om3.*t5;
t20 = R3_1.*r3;
t32 = R3_3.*r1;
t21 = t20-t32;
t22 = g.*m.*t21;
t31 = Om1.*t13;
t23 = t19+t22-t31;
t24 = Om3.*t9;
t25 = R3_2.*r3;
t34 = R3_3.*r2;
t26 = t25-t34;
t27 = g.*m.*t26;
t33 = Om2.*t13;
t28 = t24+t27-t33;
fVec = [-Om2.*R1_3+Om3.*R1_2;-Om2.*R2_3+Om3.*R2_2;-Om2.*R3_3+Om3.*R3_2;Om1.*R1_3-Om3.*R1_1;Om1.*R2_3-Om3.*R2_1;Om1.*R3_3-Om3.*R3_1;-Om1.*R1_2+Om2.*R1_1;-Om1.*R2_2+Om2.*R2_1;-Om1.*R3_2+Om2.*R3_1;Jinv1_3.*t18-Jinv1_2.*t23+Jinv1_1.*t28;Jinv2_3.*t18-Jinv2_2.*t23+Jinv2_1.*t28;Jinv3_3.*t18-Jinv3_2.*t23+Jinv3_1.*t28];
if nargout > 1
    gVec = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_1,Jinv2_1,Jinv3_1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_2,Jinv2_2,Jinv3_2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_3,Jinv2_3,Jinv3_3],[12,3]);
end
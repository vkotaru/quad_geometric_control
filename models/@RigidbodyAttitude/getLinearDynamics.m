function [A,B] = getLinearDynamics(obj,in2,in3,in4)
%GETLINEARDYNAMICS
%    [A,B] = GETLINEARDYNAMICS(OBJ,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Jan-2019 18:21:25

J1_1 = in4(3,:);
J1_2 = in4(6,:);
J1_3 = in4(9,:);
J2_1 = in4(4,:);
J2_2 = in4(7,:);
J2_3 = in4(10,:);
J3_1 = in4(5,:);
J3_2 = in4(8,:);
J3_3 = in4(11,:);
Jinv1_1 = in4(12,:);
Jinv1_2 = in4(15,:);
Jinv1_3 = in4(18,:);
Jinv2_1 = in4(13,:);
Jinv2_2 = in4(16,:);
Jinv2_3 = in4(19,:);
Jinv3_1 = in4(14,:);
Jinv3_2 = in4(17,:);
Jinv3_3 = in4(20,:);
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
t2 = J3_3.*Om3;
t3 = J1_1.*Om1;
t4 = J2_2.*Om2;
t5 = J2_1.*Om1.*2.0;
t6 = J2_3.*Om3;
t29 = J1_1.*Om2;
t7 = t4+t5+t6-t29;
t8 = J3_1.*Om1.*2.0;
t9 = J3_2.*Om2;
t30 = J1_1.*Om3;
t10 = t2+t8+t9-t30;
t11 = J2_1.*Om3;
t31 = J3_1.*Om2;
t12 = t11-t31;
t13 = J1_2.*Om2.*2.0;
t14 = J1_3.*Om3;
t32 = J2_2.*Om1;
t15 = t3+t13+t14-t32;
t16 = J3_1.*Om1;
t17 = J3_2.*Om2.*2.0;
t33 = J2_2.*Om3;
t18 = t2+t16+t17-t33;
t19 = J1_2.*Om3;
t34 = J3_2.*Om1;
t20 = t19-t34;
t21 = J1_2.*Om2;
t22 = J1_3.*Om3.*2.0;
t35 = J3_3.*Om1;
t23 = t3+t21+t22-t35;
t24 = J2_1.*Om1;
t25 = J2_3.*Om3.*2.0;
t36 = J3_3.*Om2;
t26 = t4+t24+t25-t36;
t27 = J1_3.*Om2;
t37 = J2_3.*Om1;
t28 = t27-t37;
A = reshape([0.0,0.0,0.0,-Om3,0.0,0.0,Om2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-Om3,0.0,0.0,Om2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-Om3,0.0,0.0,Om2,0.0,0.0,0.0,Om3,0.0,0.0,0.0,0.0,0.0,-Om1,0.0,0.0,0.0,0.0,0.0,0.0,Om3,0.0,0.0,0.0,0.0,0.0,-Om1,0.0,0.0,0.0,0.0,0.0,0.0,Om3,0.0,0.0,0.0,0.0,0.0,-Om1,0.0,0.0,0.0,-Om2,0.0,0.0,Om1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-Om2,0.0,0.0,Om1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-Om2,0.0,0.0,Om1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,R1_3,R2_3,R3_3,-R1_2,-R2_2,-R3_2,-Jinv1_3.*t7+Jinv1_2.*t10+Jinv1_1.*t12,-Jinv2_3.*t7+Jinv2_2.*t10+Jinv2_1.*t12,-Jinv3_3.*t7+Jinv3_2.*t10+Jinv3_1.*t12,-R1_3,-R2_3,-R3_3,0.0,0.0,0.0,R1_1,R2_1,R3_1,Jinv1_3.*t15-Jinv1_1.*t18-Jinv1_2.*t20,Jinv2_3.*t15-Jinv2_1.*t18-Jinv2_2.*t20,Jinv3_3.*t15-Jinv3_1.*t18-Jinv3_2.*t20,R1_2,R2_2,R3_2,-R1_1,-R2_1,-R3_1,0.0,0.0,0.0,-Jinv1_2.*t23+Jinv1_1.*t26+Jinv1_3.*t28,-Jinv2_2.*t23+Jinv2_1.*t26+Jinv2_3.*t28,-Jinv3_2.*t23+Jinv3_1.*t26+Jinv3_3.*t28],[12,12]);
if nargout > 1
    B = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_1,Jinv2_1,Jinv3_1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_2,Jinv2_2,Jinv3_2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Jinv1_3,Jinv2_3,Jinv3_3],[12,3]);
end

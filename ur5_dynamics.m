close all;
clear all;
clc;

% Construct kinematic model with DH Parameters [alpha, a, d, theta, offset]
lz1=0.089159;
lx3 = -0.425;
lx4 = -0.39225;
lz4 = 0.10915;
ly5 = -0.09465;
ly6 = 0.0823;

j1 = RevoluteMDH('d',lz1,'offset',pi);
j1.qlim = pi/180*[-180 180];
j2 = RevoluteMDH('alpha',pi/2);
j2.qlim = pi/180*[-180 180];
j3 = RevoluteMDH('a',lx3);
j3.qlim = pi/180*[-180 180];
j4 = RevoluteMDH('a',lx4,'d',lz4);
j4.qlim = pi/180*[-180 180];
j5 = RevoluteMDH('alpha',pi/2,'d',-ly5);
j5.qlim = pi/180*[-180 180];
j6 = RevoluteMDH('alpha',-pi/2,'d',ly6);
j6.qlim = pi/180*[-180 180];

robot = SerialLink([j1, j2, j3, j4, j5, j6], 'name', 'my-ur5')
q0 = [0 0 0 0 0 0];
%robot.teach(q0);

%% DH and Inertia Parameters of UR5
syms lz1 lx3 lx4 lz4 ly5 ly6

alpha = [0,pi/2,0,0,pi/2,-pi/2];
%a = [0,0,-0.425,-0.39225,0,0];
a = [0,0,lx3,lx4,0,0];

%d = [.08916,0,0,.10915,.09465,.0823];
d = [lz1,0,0,lz4,-ly5,ly6];

%m = [3.7,8.393,2.33,1.219,1.219,.1879];
m = sym('m%d', [1 6]);

% exact COMS
com_l1 = [0;-1.93e-3;-25.61e-3];
com_l2 = [-212.5e-3;0;113.36e-3];
com_l3 = [-242.25e-3;0;26.5e-3];
com_l4 = [0;-16.34e-3;-1.8e-3];
com_l5 = [0;16.34e-3;-1.8e-3];
com_l6 = [0;0;-1.159e-3];

% approximate COMS for simplification
syms l1 l2 l3 l4
com_l1 = [0;0;0];
com_l2 = [l1;0;l2];
com_l3 = [l3;0;0];
com_l4 = [0;-l4;0];
com_l5 = [0;l4;0];
com_l6 = [0;0;0];


In_1 = [0.014972358333333331,0,0;
        0,0.014972358333333331,0;
        0,0,0.01040625];
In_2 = [0.13388583541666665,0,0;
        0,0.13388583541666665,0;
        0,0,0.0151074];
In_3 = [0.031216803515624995,0,0;
        0,0.031216803515624995,0;
        0,0,0.004095];
In_4 = [0.002013889583333333,0,0;
        0,0.002013889583333333,0;
        0,0,0.0021942];
In_5 = [0.0018310395833333333,0,0;
        0,0.0018310395833333333,0;
        0,0,0.0021942];
In_6 = [8.062475833333332e-05,0,0;
        0,8.062475833333332e-05,0;
        0,0,0.0001321171875];

%% PARAMETERs AND SYMBOLs
g=sym('g');
alpha_0=alpha(1);alpha_1=alpha(2);alpha_2=alpha(3);alpha_3=alpha(4);alpha_4=alpha(5);alpha_5=alpha(6);
a_0=a(1);a_1=a(2);a_2=a(3);a_3=a(4);a_4=a(5);a_5=a(6);
q_1=sym('q_1');q_2=sym('q_2');q_3=sym('q_3');q_4=sym('q_4');q_5=sym('q_5');q_6=sym('q_6');
dq_1=sym('dq_1');dq_2=sym('dq_2');dq_3=sym('dq_3');dq_4=sym('dq_4');dq_5=sym('dq_5');dq_6=sym('dq_6');

%% TRANSFORMATION MATRICES AND FORWARD KINEMATICS
syms T_MDH(alpha, a, d, theta)
T_MDH(alpha, a, d, theta) = [cos(theta), -sin(theta), 0, a;
                             sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
                             sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha), cos(alpha)*d;
                             0, 0, 0, 1];


T_1 = T_MDH(0,0,lz1,pi+q_1);
T_2 = T_MDH(pi/2,0,0,q_2);
T_3 = T_MDH(0,lx3,0,q_3);
T_4 = T_MDH(0,lx4,lz4,q_4);
T_5 = T_MDH(pi/2,0,-ly5,q_5);
T_6 = T_MDH(-pi/2,0,ly6,q_6);

T_01 = T_1;
T_02 = T_01*T_2;
T_03 = T_02*T_3;
T_04 = T_03*T_4;
T_05 = T_04*T_5;
T_06 = T_05*T_6;
T = T_06;

%% ROTATION MATRICEs
R_1=T_1(1:3,1:3);
R_2=T_2(1:3,1:3);
R_3=T_3(1:3,1:3);
R_4=T_4(1:3,1:3);
R_5=T_5(1:3,1:3);
R_6=T_6(1:3,1:3);

%% COMs' POSITION VECTORs
p_c1=T_01*[com_l1; 1];
p_c1=p_c1(1:3);
p_c2=T_02*[com_l2; 1];
p_c2=p_c2(1:3);
p_c3=T_03*[com_l3; 1];
p_c3=p_c3(1:3);
p_c4=T_04*[com_l4; 1];
p_c4=p_c4(1:3);
p_c5=T_05*[com_l5; 1];
p_c5=p_c5(1:3);
p_c6=T_06*[com_l6; 1];
p_c6=p_c6(1:3);

%% SYSTEM's STATEs
q=[q_1;q_2;q_3;q_4;q_5;q_6];
dq=[dq_1;dq_2;dq_3;dq_4;dq_5;dq_6];

%% LINEAR PART of JACOBIANs
digits(4);
J_v1=jacobian(p_c1,q);
J_v2=jacobian(p_c2,q);
J_v3=jacobian(p_c3,q);
J_v4=jacobian(p_c4,q);
J_v5=jacobian(p_c5,q);
J_v6=jacobian(p_c6,q);
% J_v1=jacobian(p_c1,q);
% J_v2=jacobian(p_c2,q);
% J_v3=jacobian(p_c3,q);
% J_v4=jacobian(p_c4,q);
% J_v5=jacobian(p_c5,q);
% J_v6=jacobian(p_c6,q);

%% ROTATION MATRICEs FROM BASE
R_02=R_1*R_2;
R_03=R_02*R_3;
R_04=R_03*R_4;
R_05=R_04*R_5;
R_06=R_05*R_6;

%% ANGULAR PART of JACOBIANs
J_o1=vpa(simplify([R_1(:,3),zeros(3,5)]));
J_o2=vpa(simplify([R_1(:,3),R_02(:,3),zeros(3,4)]));
J_o3=vpa(simplify([R_1(:,3),R_02(:,3),R_03(:,3),zeros(3,3)]));
J_o4=vpa(simplify([R_1(:,3),R_02(:,3),R_03(:,3),R_04(:,3),zeros(3,2)]));
J_o5=vpa(simplify([R_1(:,3),R_02(:,3),R_03(:,3),R_04(:,3),R_05(:,3),zeros(3,1)]));
J_o6=vpa(simplify([R_1(:,3),R_02(:,3),R_03(:,3),R_04(:,3),R_05(:,3),R_06(:,3)]));

%% JACOBIAN MATRIX OF THE END-EFFECTOR
Jacobi = [J_v6;J_o6];

%% ROBOT's INERTIA (MASS) MATRIX
% eq. (6.53) Mark Spong
% Neglect last two joints for simplicity
M=simplify(J_v1.'*m(1)*J_v1)...
  +simplify(J_v2.'*m(2)*J_v2)...
  +simplify(J_v3.'*m(3)*J_v3)...
  +simplify(J_v4.'*m(4)*J_v4)...
  +simplify(J_v5.'*m(5)*J_v5)...
  +simplify(J_v6.'*m(6)*J_v6);


% M=J_v1.'*m(1)*J_v1 + J_o1.'*R_1*In_1*R_1.'*J_o1...
%  +J_v2.'*m(2)*J_v2 + J_o2.'*R_02*In_2*R_02.'*J_o2...
%  +J_v3.'*m(3)*J_v3 + J_o3.'*R_03*In_3*R_03.'*J_o3...
%  +J_v4.'*m(4)*J_v4 + J_o4.'*R_04*In_4*R_04.'*J_o4...
%  +J_v5.'*m(5)*J_v5 + J_o5.'*R_05*In_5*R_05.'*J_o5...
%  +J_v6.'*m(6)*J_v6 + J_o6.'*R_06*In_6*R_06.'*J_o6;

%% CORIOLIS and CENTRIFUGAL MATRIX
% Neglect this - it's too complicated
% C = sym('C%d%d', [6 6]);
% for k=1:6
%     for j=1:6
%         % for i=1:4
%             % eqn. (6.67) from Mark Spong et. al:
%             C(k,j) = .5*((diff(M(k,j),q(1)) + diff(M(k,1),q(j)) - diff(M(1,j),q(k)))*dq(1)...
%                          +(diff(M(k,j),q(2)) + diff(M(k,2),q(j)) - diff(M(2,j),q(k)))*dq(2)...
%                          +(diff(M(k,j),q(3)) + diff(M(k,3),q(j)) - diff(M(3,j),q(k)))*dq(3)...
%                          +(diff(M(k,j),q(4)) + diff(M(k,4),q(j)) - diff(M(4,j),q(k)))*dq(4)...
%                          +(diff(M(k,j),q(5)) + diff(M(k,5),q(j)) - diff(M(5,j),q(k)))*dq(5)...
%                          +(diff(M(k,j),q(6)) + diff(M(k,6),q(j)) - diff(M(6,j),q(k)))*dq(6));
%         % end
%     end
% end


M=vpa(simplify(M))
%V=vpa(simplify(C*dq))

%% POTENTIAL ENERGIES and GRAVITY VECTOR
% eqs. (6.64) and (6.68) from Mark Spong et. al.:
P1=m(1)*[0,0,g]*p_c1;
P2=m(2)*[0,0,g]*p_c2;
P3=m(3)*[0,0,g]*p_c3;
P4=m(4)*[0,0,g]*p_c4;
P5=m(5)*[0,0,g]*p_c5;
P6=m(6)*[0,0,g]*p_c6;
P=P1+P2+P3+P4+P5+P6;
g_1=diff(P,q_1);
g_2=diff(P,q_2);
g_3=diff(P,q_3);
g_4=diff(P,q_4);
g_5=diff(P,q_5);
g_6=diff(P,q_6);
G=vpa(simplify([g_1;g_2;g_3;g_4;g_5;g_6]))

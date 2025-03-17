clear; close all; clc

%{
UR5 Model
RRRRRR
Throwing Angles: 30 - 45 degrees

Denavit-Hartenberg (DH)
(simplified trajectory planning for ROS)
Link length (a) – Distance between two consecutive joints.
Link twist (α) – Rotation around the x-axis.
Link offset (d​) – Distance along the z-axis.
Joint angle (θ) – Rotation about the z-axis.

https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
%}

syms q1 q2 d3 q4 q5 d6 real

L = [0, 0, 0, 0, 0, 0]; %mm
m = [3.7, 8.393, 2.33, 1.219, 1.219, 0.1879]; %kg


x = 1.986;
y = 3.117;
phi = 80; %degrees to radians



%% STEP 1: Determine Initial Joint Positions 
% th = [80; -45; -45; -40; -75; 0]; %degrees
% th = [80; -45; -45; -40; 50; 0]; %degrees
th = [80; -45; -45; -40; 0; 0]; %degrees

ur5_model = loadrobot('universalUR5', 'DataFormat', 'column');
show(ur5_model, deg2rad(th));
axis equal;

% Download Peter Corke's Robotics Toolbox UR5 model
% RBE 501 WK 10
figure;
mdl_ur5;
ur5.teach

%% STEP 2: Determine joint positions using desired theta values

%{ 
DH Parameters [theta, d, a, alpha, offset]
The model provides a DH chart


UR5 [Universal Robotics]:: 6 axis, RRRRRR, stdDH, slowRNE        
+---+-----------+-----------+-----------+-----------+-----------+
| j |     theta |         d |         a |     alpha |    offset |
+---+-----------+-----------+-----------+-----------+-----------+
|  1|         q1|   0.089459|          0|     1.5708|          0|
|  2|         q2|          0|     -0.425|          0|          0|
|  3|         q3|          0|   -0.39225|          0|          0|
|  4|         q4|    0.10915|          0|     1.5708|          0|
|  5|         q5|    0.09465|          0|    -1.5708|          0|
|  6|         q6|     0.0823|          0|          0|          0|
+---+-----------+-----------+-----------+-----------+-----------+
%}

ur5

% Compute the Forward Kinematics using desired joint angles
disp('FK Dynamic Model:')
T = ur5.fkine(th) %FK dynamic model
FK = T.T
disp('Rotation Matrix:')
R = FK(1:3,1:3) % Rotation
disp('Position Matrix:')
P = FK(1:3, 4) % Position

%determine if desired position is in singularity
%non-zero value means no singularity
J_qv = ur5.jacob0(th);

if det(J_qv) == 0
    disp('Singularity')
else 
    disp('Not in Singularity')
end 

x_dot_th1 = diff(FK(1,1),q1)
x_dot_th2 = diff(FK(1,1),q2)
y_dot_th1 = diff(FK(2,1),q1)
y_dot_th2 = diff(FK(2,1),q2)
%% Step 3: Determine desired end effector position than use IK to find desired joint positions
% Inverse Kinematics - Algebraic Method
%
disp("Algebraic Solution")
% [th1_alg, th2_alg, th3_alg] = Alg_angle(x,y,L(1,1),L(2,1),L(3,1),phi)

% compute Jacobian matrix
J = ur5.jacob0([q1 q2 d3 q4 q5 d6])

q1 = ur5.ikine(FK, 'mask', [1 1 1 0 0 0]);
q2 = ur5.ikine(FK, 'mask', [1 1 1 0 0 0]);
q5 = ur5.ikine(FK, 'mask', [0 0 0 1 1 1]);
q6 = ur5.ikine(FK, 'mask', [0 0 0 1 1 1]);

figure;
ur5.plot(q1)
ur5.plot(q5)
ur5.plot(q2)
ur5.plot(q6)

%%
function [th1, th2, th3] = Alg_angle(x, y, L1, L2, L3, phi)
   if (phi >= 0)
        c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2);
        s2 = sqrt(1-c2^2);
        %s2_neg = -sqrt(1-c2^2);
        k2 = L2*s2;
        k1 = L1 + (L2*c2);
        % angle calculations
        th1 = round(rad2deg(atan2(y,x)-atan2(k2,k1)));
        th2 = round(rad2deg(atan2(s2,c2)));
        %th2_neg = round(rad2deg(atan2(s2_neg,c2)));
        th3 = round(phi-th1-th2);
   else
        c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2);
        %s2 = sqrt(1-c2^2);
        s2_neg = -sqrt(1-c2^2);
        k2 = L2*s2_neg;
        k1 = L1 + (L2*c2);
        % angle calculations
        th1 = round(rad2deg(atan2(y,x)-atan2(k2,k1)));
        %th2 = round(rad2deg(atan2(s2,c2)));
        th2 = round(rad2deg(atan2(s2_neg,c2))); %negative
        th3 = round(phi-th1-th2);
    end
end

%% Lagrange
% L = K - P

% J = manip(l1, l2, l3, q1, q2, q3);
% 
% 
% % FW Kinematics
% c1 = cos(th1);
% c2 = cos(th2);
% c123 = cos(th1 + th2 + th3);
% c23 = cos(th2 + th3);
% s1 = sin(th1);
% s2 = sin(th2);
% s23 = sin(th2 + th3);
% 
% %vertical location
% 
% Px = l2*c1 + l3*c1;
% Py = l2*c1 + l3*c1;
% Pz = l1 + l2*s2 + l3*s23;
% 
% p = [Px ; Py ; Pz];
% 
% % Potential Energy
% % P = m * g * y
% for i = 1:length(l)
%     P(i) = m(i) * g(i) * p(i)
% end
% 
% % K = (1/2)* m1 * v1^2 = 0.5 * m1 * l1^2 * th1^2
% 
% 
% % v1 = x1_dot + y1_dot = [x y][q_dot] = J * q_dot
% 
% v1 = p(1,:)*q1_d;
% v2 = p(2,:)*q2_d;
% v3 = p(3,:)*q3_d;
% 
% K1 = 0.5 * m1 * v1^2;
% K2 = 0.5 * m2 * v2^2;
% K3 = 0.5 * m3 * v3^2;
% 
% K = K1 + K2 + K3
% 
% % calculate Lagrangian
% L = K - P

%% State Space (Optimal Control)
% joint-space trajectory
% q_l = jtraj(q1_l, q2_l, t);
% q_r = jtraj(q1_r, q2_r, t);
% 
% % joint velocity and acceleration vectors, as a function of time
% [q_l, qd, qdd] = jtraj(q1_l, q2_l, t);
% [q_r, qd, qdd] = jtraj(q1_r, q2_r, t);
% 
% % jtraj method of the SerialLink class is more concise then the above step
% q_r = right.jtraj(T1, T2, t);
% q_l = left.jtraj(T1, T2, t);
% 
% % Show the trajectory as an animation
%% PID
% WK 13 RBE 500
%(2*x_dd) + (8*x_d) + (16*x) = f
% m = 2;
% b = 8;
% k = 16;
% 
% sys = tf([1],[m,b,k]);  % transfer function
% pd = pidtune(sys,'pd'); % pd = Kp + Kd * s
% 
% pidTuner(sys,pd)


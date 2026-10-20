% Reg No: EG/2020/4164
% Name:   Ravinthiran.M


%% Question 01 -> PART A

% Q3

visualizeRRRRobot(0, 0, 0, 0);


% Q5

theta1 = pi/2;
theta2 = 0;
theta3 = 0;
theta4 = pi/6;

Jacob_M_Mat = [(-0.4e0 * cos(theta2) * cos(theta3) - 0.5e0 * cos(theta2) + 0.4e0 * sin(theta2) * sin(theta3)) * sin(theta1) (-0.4e0 * cos(theta3) - 0.5e0) * cos(theta1) * sin(theta2) - 0.4e0 * cos(theta1) * cos(theta2) * sin(theta3) -0.4e0 * cos(theta1) * (sin(theta2) * cos(theta3) + cos(theta2) * sin(theta3)) 0; cos(theta1) * (0.4e0 * cos(theta3) + 0.5e0) * cos(theta2) - 0.4e0 * cos(theta1) * sin(theta2) * sin(theta3) sin(theta1) * (-0.4e0 * sin(theta2) * cos(theta3) - 0.4e0 * cos(theta2) * sin(theta3) - 0.5e0 * sin(theta2)) -0.4e0 * sin(theta1) * (sin(theta2) * cos(theta3) + cos(theta2) * sin(theta3)) 0; 0 (0.4e0 * cos(theta3) + 0.5e0) * cos(theta2) - 0.4e0 * sin(theta2) * sin(theta3) -0.4e0 * sin(theta2) * sin(theta3) + 0.4e0 * cos(theta2) * cos(theta3) 0; cos(theta1) 0 0 0; sin(theta1) 0 0 0; 0 0 0 0;];


fprintf('\nJacobian Matrix:\n');

disp(Jacob_M_Mat)

% Q6

Jacob_M_Mat_T = Jacob_M_Mat';

Mult = Jacob_M_Mat * Jacob_M_Mat_T;

Manupulability = sqrt(det(Mult));

fprintf('\nManupulability:\n');

disp(Manupulability)

% Q7

Joint_Velocity_1  = [-0.9;0.1;-0.15;0.4];

Endeffector_Velocity_1 = Jacob_M_Mat * Joint_Velocity_1;

fprintf('\nEnd-effector velocity [vx; vy; vz; wx; wy; wz]:\n');

disp(Endeffector_Velocity_1)

% Q8

Endeffector_Velocity_2 = [0.05;0.1;0;2;0;0];

Inverse_Jac = pinv(Jacob_M_Mat);

Joint_Velocity_2 = Inverse_Jac * Endeffector_Velocity_2;

fprintf('\nRequired joint velocities for desired end-effector motion:\n');

disp(Joint_Velocity_2);


%% Question 01 -> PART B

% --- Symbolic Variables ---
syms q1 q2 q3 q4 dq1 dq2 dq3 dq4 ddq1 ddq2 ddq3 ddq4 real
syms m1 m2 m3 m4 I1 I2 I3 I4 g real
%syms l1 l2 l3 real

q  = [q1; q2; q3; q4];
dq = [dq1; dq2; dq3; dq4];
ddq = [ddq1; ddq2; ddq3; ddq4];

% --- Link Lengths ---
l1 = 0.6;
l2 = 0.5; 
l3 = 0.4;


% --- Homogeneous Transformation Matrices ---
T01 = [cos(q1), 0, sin(q1), 0;
       sin(q1), 0, -cos(q1), 0;
       0, 1, 0, 0.6;
       0, 0, 0, 1];

T12 = [cos(q2), -sin(q2), 0, l2*cos(q2);
       sin(q2),  cos(q2), 0, l2*sin(q2);
       0, 0, 1, 0;
       0, 0, 0, 1];

T23 = [cos(q3), -sin(q3), 0, l3*cos(q3);
       sin(q3),  cos(q3), 0, l3*sin(q3);
       0, 0, 1, 0;
       0, 0, 0, 1];

T34 = [cos(q4), -sin(q4), 0, 0;
       sin(q4),  cos(q4), 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];

% --- Cumulative Transforms ---
T02 = simplify(T01 * T12);
T03 = simplify(T02 * T23);
T04 = simplify(T03 * T34);

% --- Center of Mass Positions ---
p_com1 = [0; 0; 0.3; 1];        % Half of d1 = 0.6
p_com2 = [0.25; 0; 0; 1];
p_com3 = [0.2; 0; 0; 1];
p_com4 = [0; 0; 0; 1];          % COM at joint 4 origin

% disp('Center of Mass position vectors:');
r1 = simplify(T01 * p_com1);
r2 = simplify(T02 * p_com2);
r3 = simplify(T03 * p_com3); 
r4 = simplify(T04 * p_com4); 

% --- Linear Velocity Jacobians ---
% disp('Linear velocity Jacobians:');
Jv1 = jacobian(r1, q);
Jv2 = jacobian(r2, q);
Jv3 = jacobian(r3, q);
Jv4 = jacobian(r4, q);
% will likely have non-zero rows due to rotation chain
%Jv4 = sym(zeros(3,4))

% --- Angular Velocity Jacobians ---
z0 = [0; 0;0; 1];
z1 = T01(1:4,4);
z2 = T02(1:4,4);
z3 = T03(1:4,4);
zero = [0;0;0;0];

% disp('Angular velocity Jacobians:');
Jw1 = [z0, zero,zero,zero];
Jw2 = [z0, z1, zero,zero];
Jw3 = [z0, z1, z2, zero];
Jw4 = [z0, z1, z2, z3];

% --- Kinetic Energy ---
%disp('Kinetic energy of Link 1:');
v1 = Jv1*dq;
w1 = Jw1*dq;
T1 = 0.5*m1*v1.'*v1 + 0.5*I1*w1.'*w1;
% disp('Kinetic energy of Link 2:');
v2 = Jv2*dq;
w2 = Jw2*dq;
T2 = 0.5*m2*v2.'*v2 + 0.5*I2*w2.'*w2;
% disp('Kinetic energy of Link 3:');
v3 = Jv3*dq;
w3 = Jw3*dq;
T3 = 0.5*m3*v3.'*v3 + 0.5*I3*w3.'*w3;
% disp('Kinetic energy of Link 4:');
v4 = Jv4*dq;
w4 = Jw4*dq;
T4 = 0.5*m4*v4.'*v4 + 0.5*I4*w4.'*w4;

%disp('Total Kinetic energy:');
T_total = simplify(T1 + T2 + T3 + T4);


% --- Potential Energy ---
%disp('Potential energy:');
gravity = [0; 0; -g; 0];
V_1 = m1 * gravity.' * r1;
V_2 = m2 * gravity.' * r2;
V_3 = m3 * gravity.' * r3;
V_4 = m4 * gravity.' * r4;
V_total = simplify(V_1 + V_2 + V_3 + V_4);

% --- Lagrangian ---
L = T_total - V_total;

% --- Equations of Motion using Euler-Lagrange ---
eqn = sym('eqn', [4 1]);

for i = 1:4
    dLdq = diff(L, q(i));
    dLdqdot = diff(L, dq(i));
    ddt_dLdqdot = 0;
    for j = 1:4
        ddt_dLdqdot = ddt_dLdqdot + diff(dLdqdot, q(j)) * dq(j) + diff(dLdqdot, dq(j)) * sym(['ddq' num2str(j)]);
    end
    eqn(i) = simplify(ddt_dLdqdot - dLdq);
end


disp('Symbolic Equations of Motion: M(q)ddq + C(q,dq)dq + G(q)');
eqn = simplify(eqn, 'Steps', 1000);

% Collect terms
eqn_collected = collect(eqn, [ddq1, ddq2, ddq3, ddq4, dq1, dq2, dq3, dq4]);

disp('Collected Equations of Motion:');
pretty(eqn_collected)






%% Question 2

% Time setup
t1 = linspace(0, 5, 100);        % Segment 1: 0 to 5 seconds
t2 = linspace(0, 5, 100);        % Segment 2: 0 to 5 seconds (starts at t=5)

% === Segment 1: 20° to 80° in 5s ===
theta0_1 = 20; thetaf_1 = 80;
v0_1 = 0; vf_1 = 0;
T1 = 5;

% Coefficient matrix
A = [1 0 0      0;
     1 T1 T1^2  T1^3;
     0 1 0      0;
     0 1 2*T1   3*T1^2];

b1 = [theta0_1; thetaf_1; v0_1; vf_1];
coeff1 = A\b1;

% Display coefficients for Segment 1
fprintf('Segment 1 Coefficients (20° to 80° in 5s):\n');
fprintf('a0 = %.4f\n', coeff1(1));
fprintf('a1 = %.4f\n', coeff1(2));
fprintf('a2 = %.4f\n', coeff1(3));
fprintf('a3 = %.4f\n\n', coeff1(4));

syms t
theta_poly1 = coeff1(1) + coeff1(2)*t + coeff1(3)*t^2 + coeff1(4)*t^3;
disp('Segment 1 Polynomial:');
pretty(simplify(theta_poly1))

coeff1 = A \ b1;   % For segment 1

% Position, Velocity, Acceleration
theta1 = polyval(flip(coeff1'), t1);
vel1   = coeff1(2) + 2*coeff1(3)*t1 + 3*coeff1(4)*t1.^2;
acc1   = 2*coeff1(3) + 6*coeff1(4)*t1;

% === Segment 2: 80° to 25° in 5s ===
theta0_2 = 80; thetaf_2 = 25;
v0_2 = 0; vf_2 = 0;
T2 = 5;

b2 = [theta0_2; thetaf_2; v0_2; vf_2];
coeff2 = A\b2;

% Display coefficients for Segment 2
fprintf('Segment 2 Coefficients (80° to 25° in 5s):\n');
fprintf('a0 = %.4f\n', coeff2(1));
fprintf('a1 = %.4f\n', coeff2(2));
fprintf('a2 = %.4f\n', coeff2(3));
fprintf('a3 = %.4f\n\n', coeff2(4));

% Position, Velocity, Acceleration
theta2 = polyval(flip(coeff2'), t2);
vel2   = coeff2(2) + 2*coeff2(3)*t2 + 3*coeff2(4)*t2.^2;
acc2   = 2*coeff2(3) + 6*coeff2(4)*t2;


syms t
tau = t - 5;
theta_poly2 = coeff2(1) + coeff2(2)*tau + coeff2(3)*tau^2 + coeff2(4)*tau^3;
disp('Segment 2 Polynomial:');
pretty(simplify(theta_poly2))

% === Combine for full 0–10s time ===
t_full = [t1, t2+5];
theta_full = [theta1, theta2];
vel_full   = [vel1, vel2];
acc_full   = [acc1, acc2];

% === Plotting ===
figure;

subplot(3,1,1);
plot(t_full, theta_full, 'b', 'LineWidth', 2);
ylabel('Angle (°)');
title('Joint Angle vs Time');
grid on;

subplot(3,1,2);
plot(t_full, vel_full, 'r', 'LineWidth', 2);
ylabel('Velocity (°/s)');
title('Joint Velocity vs Time');
grid on;

subplot(3,1,3);
plot(t_full, acc_full, 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (°/s²)');
title('Joint Acceleration vs Time');
grid on;





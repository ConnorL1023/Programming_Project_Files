close all
clc

%% initialization of variables
dt = .01;
box_size = 60;
in_to_m = .0254;
L = in_to_m*[13,13];
m_load = 20;

q1_init = -pi/2;
q2_init = pi/4;
q_init = [q1_init, q2_init];
x_init = L(1)*cos(q1_init) + L(2)*(cos(q1_init + q2_init));
y_init = L(1)*sin(q1_init) + L(2)*(sin(q1_init + q2_init));
pe_init = [x_init y_init];

plot([0, round(L(1)*cos(q1_init),5), round(x_init,5)], [0, L(1)*sin(q1_init), y_init], 'b', 'LineWidth', 4)

t = 0;
t_list = [0];

q1_current = q1_init;
q2_current = q2_init;
qd1_current = 0;
qd2_current = 0;

% %% joint controller - comment whole section in to use
% % used for the generic PD loop
% q1_desired = pi/4;
% q2_desired = 0;
% q_desired = [q1_desired q2_desired];
% x_desired = .6604*2^.5;
% y_desired = .6604*2^.5;
% pe_desired = [x_desired y_desired];
% 
% q1_error = q1_desired - q1_init;
% q2_error = q2_desired - q2_init;
% 
% previous_q1_error = 0;
% previous_q2_error = 0;
% 
% x_current = x_init;
% y_current = x_init;
% 
% while (abs(q1_error) > 0.001 || abs(q2_error)> 0.001)
%     Kp = 0.05;
%     Kd = 0.01;
% 
%     q1_error = q1_desired - q1_current;
%     q2_error = q2_desired - q2_current;
% 
%     q1_error_derivative = q1_error - previous_q1_error;
%     q2_error_derivative = q2_error - previous_q2_error;
% 
%     q1_current = q1_current + Kp*q1_error + Kd*q2_error_derivative;
%     q2_current = q2_current + Kp*q2_error + Kd*q2_error_derivative;
% 
%     previous_q1_error = q1_error;
%     previous_q2_error = q2_error;
% 
%     x_current = L(1)*cos(q1_current) + L(2)*(cos(q1_current + q2_current));
%     y_current = L(1)*sin(q1_current) + L(2)*(sin(q1_current + q2_current));
%    
%     cla
%     plot(x_current, y_current,'rs','MarkerSize', box_size, 'MarkerFaceColor','r')
%     hold on
%     plot([0, round(L(1)*cos(q1_current),5), round(x_current,5)], [0, round(L(2)*sin(q1_current),5), round(y_current,5)], 'b', 'LineWidth', 4)
%     axis([-1, 1, -1, 1])
%     pause(0.01)
% end

%% force controller - comment whole section in to use
F_desired = [0, 0];
% the force is negative because its a reaction force, the one felt by the robot
% uncomment one of the following lines, paired with the line in the for loop, to
% simulate
% F_ext_current = [0, -10];              % step input
% F_ext_current = [0,  2.5*t - 10];          % ramp input
F_ext_current = [0, (t - 2)^2 - 10];  % inverse parabola input
F_ext_list = [F_ext_current];

p_current = pe_init;
dF = F_desired - F_ext_current;
Kp = 0.0005;
Ki = 0.0001;
Kd = 0.0001;
pos_error = Kp*(dF + Ki*dF);
torques = [0;0];

while (abs(pos_error(1)) > 0.0001 || abs(pos_error(2))> 0.0001)
    t = t+dt;
    t_list = [t_list; t];
%     F_ext_current = [0, -10];              % step input
%     F_ext_current = [0,  2.5*t - 10];          % ramp input
    F_ext_current = [0, (2*t - 2)^2 - 10]; % inverse parabola input
    if (F_ext_current(2) >= 0)
        F_ext_current = [0, 0];
    end
    F_ext_list = [F_ext_list; F_ext_current];
    dF = F_desired - F_ext_current;
    p_desired = p_current + Kp*(dF + Ki*dF);
    pos_error = p_desired - p_current;
    
    q1_prev = q1_current;
    q2_prev = q2_current;
    q2_current = acos((p_desired(1)^2 + p_desired(2)^2 - L(1)^2 - L(2)^2)/(2*L(1)*L(2)));
    q1_current = atan2(p_desired(2), p_desired(1)) - atan2(L(2)*sin(q2_current), L(1)+L(2)*cos(q2_current));
    
    qd1_prev = qd1_current;
    qd2_prev = qd2_current;
    qd1_current = (q1_current-q1_prev)/dt;
    qd2_current = (q2_current-q2_prev)/dt;
    
    qdd1_current = (qd1_current-qd1_prev)/dt;
    qdd2_current = (qd2_current-qd2_prev)/dt;
    
    torques = [torques, calc_torques(q1_current, q2_current, qd1_current, qd2_current, qdd1_current, qdd2_current, F_ext_current, L, m_load)];
    
    p_current = [L(1)*cos(q1_current) + L(2)*(cos(q1_current + q2_current)),...
                 L(1)*sin(q1_current) + L(2)*(sin(q1_current + q2_current))];    
    
    cla
    plot(p_current(1), p_current(2),'rs','MarkerSize', box_size, 'MarkerFaceColor','r')
    hold on
    plot([0, round(L*cos(q1_current),5), round(p_current(1),5)], [0, round(L*sin(q1_current),5), round(p_current(2),5)], 'b', 'LineWidth', 4)
    axis([-1, 1, -1, 1])
    title('Simulation of Robotic Arm Lifting an Object')
    xlabel('X position (m)')
    ylabel('Y position (m)')
    pause(0.01)
end

% note - if the joint limits are reached, then the t_list for plotting the
% torques should be t_list(3:end-1) instead
figure()
hold on
plot(t_list(3:end), torques(1,3:end))
plot(t_list(3:end), torques(2,3:end))
legend('Joint 1 (origin)','Joint 2 (connects links)','location','northeast')
title('Joint Torques Over Time')
xlabel('Time (s)')
ylabel('Torque (N/m)')

figure()
hold on
plot(t_list, F_ext_list(:,2), 'LineWidth', 1)
axis([-0.1, t_list(end) -15 0.5])
legend('Force input (N)', 'location', 'northwest')
title('Force Curve')
xlabel('Time (s)')
ylabel('Force (N)')

%% function definitions
function torques = calc_torques(q1, q2, qd1, qd2, qdd1, qdd2, F_ext, L, m_load)
L1 = L(1); % [m]
L2 = L(2); % [m]

m1 = 3.04;
m2 = 2.66;

CoG1 = L1/2;
CoG2 = L2/2;

I1 = m1 * (L1^2) / 12;
I2 = m2 * (L2^2) / 12;

B = [I1 + m1*CoG1^2 + I2 + m2*(L1^2 + CoG2^2 + 2*L1*CoG2*cos(q2)), I2 + m2*(CoG2+L1*CoG2*cos(q2));
    I2 + m2*(CoG2+L1*CoG2*cos(q2)), I2 + m2*CoG2^2];

C = [-m2*L1*CoG2*sin(q2)*qd2, -m2*L1*CoG2*sin(q2)*(qd1 + qd2);
     m2*L1*CoG2*sin(q2)*qd1, 0];

g = -9.807;
G = [(m1*CoG1 + m2*L1)*g*cos(q1) + m2*CoG2*g*cos(q1 + q2); m2*CoG2*g*cos(q1 + q2)];

% Load Force to Torques

xdd_load = -L1*cos(q1)*qd1^2 - L1*sin(q1)*qdd1 ...
              - L2*cos(q1 + q2)*(qd1 + qd2)^2 ...
              - L2*sin(q1 + q2)*(qdd1 + qdd2);
ydd_load = -L1*sin(q1)*qd1^2 + L1*cos(q1)*qdd1 ...
              - L2*sin(q1 + q2)*(qd1 + qd2)^2 ...
              + L2*cos(q1 + q2)*(qdd1 + qdd2);
          
F_load_inertia = m_load*[xdd_load; ydd_load];
F_load_gravity = m_load*[0;g];
F_load_total = F_load_inertia+F_load_gravity;
J = [-L1*sin(q1) - L2*sin(q1 + q2), -L2*sin(q1 + q2);
    L1*cos(q1) + L2*cos(q1 + q2), L2*cos(q1 + q2)];
t_load = -J'*F_load_total;

% Torque by Human Forces

t_human = J'*F_ext';

% Joint torques
torques = B * [qdd1; qdd2] + C * [qd1; qd2] + G + t_load + t_human;

end

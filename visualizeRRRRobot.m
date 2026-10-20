%EG/2020/4164
%Ravinthiran.M

function visualizeRRRRobot(theta1, theta2, theta3, theta4)
% visualizeRRRRobot - Visualizes a 4-DOF RRRR robotic arm using HTMs
                % Visualization helper; requires MATLAB graphics support. 
                %   visualizeRRRRobot(theta1, theta2, theta3, theta4)
                % Example invocation in MATLAB command window: 
                %   visualizeRRRRobot(pi/2, 0, 0, pi/6);

    % HTM matrices from DH convention
    T01 = [cos(theta1) 0 sin(theta1) 0;
           sin(theta1) 0 -cos(theta1) 0;
           0 1 0 0.6e0;
           0 0 0 1];

    T12 = [cos(theta2), -sin(theta2), 0, 0.5 * cos(theta2);
           sin(theta2),  cos(theta2), 0, 0.5 * sin(theta2);
           0,            0,           1, 0;
           0,            0,           0, 1];

    T23 = [cos(theta3), -sin(theta3), 0, 0.4 * cos(theta3);
           sin(theta3),  cos(theta3), 0, 0.4 * sin(theta3);
           0,            0,           1, 0;
           0,            0,           0, 1];

    T34 = [cos(theta4), -sin(theta4), 0, 0;
           sin(theta4),  cos(theta4), 0, 0;
           0,            0,           1, 0;
           0,            0,           0, 1];

    % Cumulative transformations
    T0 = eye(4);
    T1 = T01;
    T2 = T1 * T12;
    T3 = T2 * T23;
    T4 = T3 * T34;

    % Positions of each joint
    positions = [T0(1:3,4)'; T1(1:3,4)'; T2(1:3,4)'; T3(1:3,4)'; T4(1:3,4)'];

    % Plotting
    figure;
    ax = gca;
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    title(ax, '4-DOF RRRR Robot Visualization');

    % Plot links
    for i = 1:4
        plot3(ax, [positions(i,1), positions(i+1,1)], ...
                  [positions(i,2), positions(i+1,2)], ...
                  [positions(i,3), positions(i+1,3)], ...
                  'b-', 'LineWidth', 5);
    end

    % Plot joints
    plot3(ax, positions(:,1), positions(:,2), positions(:,3), ...
          'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

    % Plot coordinate frames
    T_all = {T0, T1, T2, T3, T4};
    arrowScale = 0.1;
    for i = 1:5
        T = T_all{i};
        origin = T(1:3, 4);
        x_axis = T(1:3, 1);
        y_axis = T(1:3, 2);
        z_axis = T(1:3, 3);
        quiver3(origin(1), origin(2), origin(3), ...
                arrowScale*x_axis(1), arrowScale*x_axis(2), arrowScale*x_axis(3), ...
                'r', 'LineWidth', 2, 'MaxHeadSize', 1);
        quiver3(origin(1), origin(2), origin(3), ...
                arrowScale*y_axis(1), arrowScale*y_axis(2), arrowScale*y_axis(3), ...
                'g', 'LineWidth', 2, 'MaxHeadSize', 1);
        quiver3(origin(1), origin(2), origin(3), ...
                arrowScale*z_axis(1), arrowScale*z_axis(2), arrowScale*z_axis(3), ...
                'b', 'LineWidth', 2, 'MaxHeadSize', 1);
    end

    view(3);
    hold(ax, 'off');
end

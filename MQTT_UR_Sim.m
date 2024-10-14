function MQTT_UR_Sim()
    % MQTT Setup
    brokerAddress = "tcp://10.243.82.33:1883";
    ClientID = "group1sim";
    topic_angles = "group1_joint_angles";  % Topic for receiving joint angles
    topic_status = "group1_status";        % Topic for publishing status

    % Try to establish MQTT connection
    Client = connect_mqtt(brokerAddress, ClientID);

    % Publish "done" on startup to indicate the robot is ready
    write(Client, topic_status, "done");
    disp('Published "done" to robot/status on startup');

    % Subscribe to the joint_angles topic
    subscribe(Client, topic_angles);

    % Load the UR5 robot model from the Robotics System Toolbox
    figure(1);
    clf;
    robot = loadrobot('universalUR3', 'DataFormat', 'row', 'Gravity', [0, 0, -9.81]);

    % Set initial joint angles
    joint_angles = [0, -pi/2, pi/2, 0, 0, 0]; % Neutral starting position

    % Create figure for the robot visualization
    figure(1);
    show(robot, joint_angles, 'Visuals', 'on', 'Frames', 'on');
    axis([-0.5 0.5 -0.5 0.5 0 0.75]); % Set fixed axis limits
    grid on;
    title('6-DOF Robot Interactive Control');
    create_ground_plane([-0.5, 0.5], [-0.5, 0.5], [0 1 0], 0.5); % Ground size and transparency

    hold on;

    % Initialize last position to the home position
    last_angles = joint_angles;

    % Continuous loop to check for new messages
    while true
        % Check for MQTT connection
        if isempty(Client)
            disp('Reconnecting to MQTT broker...');
            Client = connect_mqtt(brokerAddress, ClientID);
            subscribe(Client, topic_angles);  % Subscribe again if disconnected
            pause(5); % Wait before retrying to connect
            continue; % Retry the loop until a connection is re-established
        end

        % Read the next available MQTT message, if any
        msg = read(Client); % Read the message
        if ~isempty(msg)
            try
                % Parse the message into joint angles
                new_angles = str2num(msg.Data); %#ok<ST2NM>
            catch
                disp('Invalid message format. Please send a vector of 6 numbers.');
                continue;
            end

            % Publish "received" to the "robot/status" topic after receiving the message
            write(Client, topic_status, "received");
            disp('Published "received" to status topic');

            % Interpolate between the last angles and the new angles using spline
            num_steps = 50; % Number of steps for smooth transition
            t = linspace(0, 1, num_steps); % Parameter for interpolation
            joint_trajectory = zeros(num_steps, 6); % Preallocate space for trajectory

            for i = 1:6
                % Use spline interpolation to generate a smooth trajectory
                joint_trajectory(:, i) = spline([0 1], [last_angles(i), new_angles(i)], t);
            end

            % Animate the robot moving from last_angles to new_angles
            for step = 1:num_steps
                % Update the robot visualization
                show(robot, joint_trajectory(step, :), 'Visuals', 'on', 'Frames', 'on', FastUpdate=true, PreservePlot=false);
                axis([-0.5 0.5 -0.5 0.5 0 0.75]); % Fixed axis limits
                axis equal;
                grid on;
                title('6-DOF Robot Interactive Control');
                pause(0.01); % Pause to control the animation speed
            end

            % Update last_angles to the new angles for future interpolation
            last_angles = new_angles;

            % Publish "done" to the "robot/status" topic after completing movement
            write(Client, topic_status, "done");
            disp('Published "done" to robot/status');
        end

        pause(0.1); % Pause to allow time for the next message
    end
end

function create_ground_plane(x_range, y_range, color, alpha)
    % Creates an opaque plane at z=0 to simulate the ground.
    % Input:
    % x_range: [min_x, max_x] - range for x-axis
    % y_range: [min_y, max_y] - range for y-axis
    % color: [R, G, B] - RGB color values for the ground
    % alpha: transparency level (0 = fully transparent, 1 = fully opaque)

    % Create the ground plane as a rectangle
    patch('XData', [x_range(1), x_range(2), x_range(2), x_range(1)], ...
          'YData', [y_range(1), y_range(1), y_range(2), y_range(2)], ...
          'ZData', [0, 0, 0, 0], ...  % Ground plane is at z = 0
          'FaceColor', color, ...
          'FaceAlpha', alpha, ... % Set the transparency
          'EdgeColor', 'none');   % No edges on the plane
end

function Client = connect_mqtt(brokerAddress, ClientID)
    % Try to connect to the MQTT broker and return the client object
    try
        Client = mqttclient(brokerAddress, ClientID=ClientID, Port=1883);
        disp('Connected to MQTT broker successfully.');
    catch
        disp('Failed to connect to MQTT broker. Retrying...');
        Client = [];
    end
end

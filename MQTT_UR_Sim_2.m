function MQTT_UR_Sim_2()
    % Define geometric parameters for layout adjustments
    params.sliderLength = 200;   % Length of the sliders
    params.sliderLabelWidth = 50; % Width of slider labels
    params.sliderDisplayWidth = 50; % Width of the slider value display
    params.controlPanelWidth = 800; % Width of the entire control panel
    params.stepSize = 0.1; % Step size for end-effector movements in meters

    % MQTT Setup
    brokerAddress = "tcp://10.243.82.33:1883";
    ClientID = "group1sim";
    topic_angles = "group1_joint_angles";         % Topic for receiving joint angles
    topic_status = "group1_status";               % Topic for publishing status
    topic_end_effector = "group1_end_effector";   % New topic for end effector position

    % Global variables and flags
    global joint_angles last_angles stopProgram sliderLock;
    joint_angles = [0, -pi/2, pi/2, 0, 0, 0]; % Neutral starting position
    last_angles = joint_angles;
    stopProgram = false;
    busy = false; % Flag to indicate if the arm is currently moving
    moveQueue = {}; % Queue to store movement commands
    sliderLock = false(1, 6); % Array to lock sliders after manual adjustments

    % Establish MQTT connection
    Client = connect_mqtt(brokerAddress, ClientID);

    % Cleanup function to handle unexpected exits
    cleanupObj = onCleanup(@() cleanExit(Client));

    % Publish "done" on startup to indicate the robot is ready
    write(Client, topic_status, "done");
    disp('Published "done" to robot/status on startup');

    % Subscribe to the joint_angles topic
    subscribe(Client, topic_angles);

    % Create main UI figure and configure a callback for when the window is closed
    fig_name = 'Robot Joint Control';
    ui_fig = uifigure('Name', fig_name, 'Position', [100 200 1200 600], ...
                      'CloseRequestFcn', @(src, event) closeProgram());

    % Configure grid layout in the figure (2 columns, 1 row)
    gl = uigridlayout(ui_fig, [1, 2]);
    gl.ColumnWidth = {'3x', '1.5x'}; % Adjust to make space for controls on the right

    % Configure axes for robot visualization
    ax = axes(gl);
    ax.Title.String = '6-DOF Robot Interactive Control';
    hold(ax, 'on');
    view(ax, 3);
    axis(ax, 'vis3d');
    rotate3d(ax, 'on');
    ax.XLim = [-0.5 0.5];
    ax.YLim = [-0.5 0.5];
    ax.ZLim = [0 0.75];

    % Load the UR3 robot model and display initial position in axes
    robot = loadrobot('universalUR3', 'DataFormat', 'row', 'Gravity', [0, 0, -9.81]);
    show(robot, joint_angles, 'Parent', ax, 'Visuals', 'on', 'Frames', 'on', 'PreservePlot', false);

    % Simple lighting adjustment
    camlight(ax, "headlight");

    % Create ground plane
    create_ground_plane([-0.5, 0.5], [-0.5, 0.5], [0 1 0], 0.5, ax);

    % Control panel on the right
    control_panel = uipanel(gl, 'Title', 'Robot Controls');
    control_layout = uigridlayout(control_panel, [2, 1]);

    % Joint angle control panel with left-justified sliders and display boxes
    slider_panel = uipanel(control_layout, 'Title', 'Joint Angle Control');
    slider_layout = uigridlayout(slider_panel, [6, 3]);
    slider_layout.ColumnWidth = {params.sliderLabelWidth, params.sliderLength, params.sliderDisplayWidth};
    sliders = gobjects(1, 6);
    slider_labels = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"];
    slider_displays = gobjects(1, 6); % To store display boxes for each slider

    % Initialize sliders with left justification and display boxes
    for i = 1:6
        uilabel(slider_layout, 'Text', slider_labels(i), 'HorizontalAlignment', 'left');
        sliders(i) = uislider(slider_layout, ...
                              'Limits', [-pi, pi], ...
                              'Value', joint_angles(i), ...
                              'MajorTicks', [-pi, -pi/2, 0, pi/2, pi], ...
                              'ValueChangedFcn', @(src, event) updateJointAngle(src, i));
        slider_displays(i) = uieditfield(slider_layout, 'numeric', 'Value', joint_angles(i), ...
                                         'Editable', 'off', 'HorizontalAlignment', 'center');
    end

    % End-effector control panel with 3x2 button layout
    arrow_pad = uipanel(control_layout, 'Title', 'End-Effector Control');
    pad_layout = uigridlayout(arrow_pad, [3, 2]);

    % Define step size for end-effector movement in meters
    step_size = params.stepSize;

    % Add buttons for up, down, forward, backward, and side-to-side control
    uibutton(pad_layout, 'Text', '+Y', 'ButtonPushedFcn', @(src, event) queueMovement([0, step_size, 0]));
    uibutton(pad_layout, 'Text', '-Y', 'ButtonPushedFcn', @(src, event) queueMovement([0, -step_size, 0]));
    uibutton(pad_layout, 'Text', '-X', 'ButtonPushedFcn', @(src, event) queueMovement([-step_size, 0, 0]));
    uibutton(pad_layout, 'Text', '+X', 'ButtonPushedFcn', @(src, event) queueMovement([step_size, 0, 0]));
    uibutton(pad_layout, 'Text', '+Z', 'ButtonPushedFcn', @(src, event) queueMovement([0, 0, step_size]));
    uibutton(pad_layout, 'Text', '-Z', 'ButtonPushedFcn', @(src, event) queueMovement([0, 0, -step_size]));

    % Function to queue end-effector movements and handle execution
    function queueMovement(delta)
        % Add movement to queue
        moveQueue{end+1} = delta;
        % Start processing if the arm is not busy
        if ~busy
            processQueue();
        end
    end

    % Function to process movements in the queue
    function processQueue()
        if isempty(moveQueue)
            busy = false;
            return;
        end

        % Set busy flag to true and get the next movement from the queue
        busy = true;
        delta = moveQueue{1};
        moveQueue(1) = []; % Remove the movement from the queue

        % Perform the movement
        move_end_effector(delta);

        % Set a callback to check the queue again after the movement is done
        busy = false;
        if ~isempty(moveQueue)
            processQueue();
        end
    end

    % Nested function to move the end effector by a small increment
    function move_end_effector(delta)
        ik = inverseKinematics('RigidBodyTree', robot);
        end_effector = 'tool0';

        % Get current end-effector position and orientation
        tform = getTransform(robot, joint_angles, end_effector);
        new_tform = tform;
        new_tform(1:3, 4) = new_tform(1:3, 4) + delta';

        % Solve IK for the new position
        [new_angles, ~] = ik(end_effector, new_tform, [1 1 1 1 1 1], joint_angles);

        % Smoothly transition to the new angles
        update_joint_angles(new_angles);
    end

    % Function to update sliders and display boxes with the latest joint angles
    function update_sliders()
        for i = 1:6
            % Update slider if not locked due to manual adjustment
            if ~sliderLock(i)
                % Wrap the angle within the range [-pi, pi]
                slider_value = mod(joint_angles(i) + pi, 2 * pi) - pi;
                sliders(i).Value = slider_value;
                slider_displays(i).Value = slider_value; % Update display box
            end
        end
    end

    % Continuous loop to check for new MQTT messages
    try
        while ishghandle(ui_fig) && ~stopProgram  % Keep running while the figure is open and stop flag is false
            if isempty(Client)
                disp('Reconnecting to MQTT broker...');
                Client = connect_mqtt(brokerAddress, ClientID);
                subscribe(Client, topic_angles);
                pause(5);
                continue;
            end

            % Read the next available MQTT message, if any
            msg = read(Client);
            if ~isempty(msg)
                try
                    new_angles = str2num(msg.Data); %#ok<ST2NM>
                    if length(new_angles) ~= 6
                        error('Invalid number of joint angles received.');
                    end
                catch
                    disp('Invalid message format. Please send a vector of 6 numbers.');
                    continue;
                end

                write(Client, topic_status, "received");
                disp('Published "received" to status topic');

                % Reset slider locks since a programmatic action is taking place
                sliderLock(:) = false;
                % Interpolate smoothly to the new angles
                update_joint_angles(new_angles);
            end

            pause(0.1);
        end
    catch ME
        disp('Program interrupted or encountered an error.');
        % Ensure clean exit if stopped unexpectedly
        cleanExit(Client);
        rethrow(ME);
    end

    % Nested function to update joint angles
    function update_joint_angles(new_angles)
        num_steps = 50;
        t = linspace(0, 1, num_steps);
        joint_trajectory = zeros(num_steps, 6);

        for i = 1:6
            joint_trajectory(:, i) = spline([0 1], [last_angles(i), new_angles(i)], t);
        end

        for step = 1:num_steps
            current_angles = joint_trajectory(step, :);
            show(robot, current_angles, 'Parent', ax, 'Visuals', 'on', 'Frames', 'on', 'FastUpdate', true, 'PreservePlot', false);
            
            % Update joint angles and sliders to reflect the current interpolation step
            joint_angles = current_angles;
            update_sliders();
            
            pause(0.01);
        end

        last_angles = new_angles;  % Set the last angles to the final position
    end

    % Callback for slider updates
    function updateJointAngle(src, joint_index)
        % Set the manual slider update lock for this specific joint
        sliderLock(joint_index) = true;
        new_angles = joint_angles;
        new_angles(joint_index) = src.Value;
        update_joint_angles(new_angles);
    end

    % Close program function
    function closeProgram()
        stopProgram = true;     % Set stop flag to exit loop
        delete(ui_fig);         % Delete the figure to close the window
    end
end

function create_ground_plane(x_range, y_range, color, alpha, ax)
    patch(ax, 'XData', [x_range(1), x_range(2), x_range(2), x_range(1)], ...
          'YData', [y_range(1), y_range(1), y_range(2), y_range(2)], ...
          'ZData', [0, 0, 0, 0], ...
          'FaceColor', color, ...
          'FaceAlpha', alpha, ...
          'EdgeColor', 'none');
end

function Client = connect_mqtt(brokerAddress, ClientID)
    try
        Client = mqttclient(brokerAddress, 'ClientID', ClientID, 'Port', 1883);
        disp('Connected to MQTT broker successfully.');
    catch
        disp('Failed to connect to MQTT broker. Retrying...');
        Client = [];
    end
end

function cleanExit(Client)
    disp('Cleaning up...');
    if ~isempty(Client)
        try
            Client.disconnect();
            disp('MQTT Client disconnected.');
        catch
            disp('MQTT Client was already disconnected.');
        end
    end
    close all force; % Ensure all figures are closed
end

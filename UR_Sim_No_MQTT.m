function UR_Sim_No_MQTT()
    % Define geometric parameters for layout adjustments
    params.sliderLength = 200;   % Length of the sliders
    params.sliderLabelWidth = 50; % Width of slider labels
    params.sliderDisplayWidth = 50; % Width of the slider value display
    params.controlPanelWidth = 800; % Width of the entire control panel
    params.stepSize = 0.1; % Step size for end-effector movements in meters

    % Global variables and flags
    global joint_angles last_angles end_effector_pos stopProgram sliderLock;
    joint_angles = [0, -pi/2, pi/2, 0, 0, 0, 0, 0]; % Initial position including end-effector joints
    last_angles = joint_angles;
    end_effector_pos = 0; % Initial end-effector position (0 = fully closed)
    stopProgram = false;
    busy = false;
    moveQueue = {};
    sliderLock = false(1, 6); % Array to lock joint sliders

    % Create main UI figure and configure a callback for when the window is closed
    fig_name = 'Robot Joint and End-Effector Control';
    ui_fig = uifigure('Name', fig_name, 'Position', [100 200 1200 600], ...
                      'CloseRequestFcn', @(src, event) closeProgram());

    % Configure grid layout in the figure (2 columns, 1 row)
    gl = uigridlayout(ui_fig, [1, 2]);
    gl.ColumnWidth = {'3x', '1.5x'}; % Adjust to make space for controls on the right

    % Configure axes for robot visualization
    ax = axes(gl);
    ax.Title.String = '6-DOF Robot with End-Effector Control';
    hold(ax, 'on');
    view(ax, 3);
    axis(ax, 'vis3d');
    rotate3d(ax, 'on');
    ax.XLim = [-0.5 0.5];
    ax.YLim = [-0.5 0.5];
    ax.ZLim = [0 0.75];

    % Load the UR3 robot model with end-effector and display initial position
    robot = loadrobot('universalUR3', 'DataFormat', 'row', 'Gravity', [0, 0, -9.81]);
    end_effector = load("robotiq_hande.mat").robotiq_hande;
    addSubtree(robot,"tool0",end_effector)
    show(robot, joint_angles, 'Parent', ax, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);

    % Simple lighting adjustment
    camlight(ax, "headlight");

    % Create ground plane
    create_ground_plane([-0.5, 0.5], [-0.5, 0.5], [0 1 0], 0.5, ax);

    % Control panel on the right
    control_panel = uipanel(gl, 'Title', 'Robot Controls');
    control_layout = uigridlayout(control_panel, [3, 1]);

    % Joint angle control panel
    slider_panel = uipanel(control_layout, 'Title', 'Joint Angle Control');
    slider_layout = uigridlayout(slider_panel, [6, 3]);
    slider_layout.ColumnWidth = {params.sliderLabelWidth, params.sliderLength, params.sliderDisplayWidth};
    sliders = gobjects(1, 6);
    slider_labels = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"];
    slider_displays = gobjects(1, 6);

    % Initialize sliders for joint angles
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

    % End-effector control slider
    effector_panel = uipanel(control_layout, 'Title', 'End-Effector Control');
    uilabel(effector_panel, 'Text', 'Grip');
    effector_slider = uislider(effector_panel, ...
                               'Limits', [0, 100], ...
                               'Value', end_effector_pos, ...
                               'MajorTicks', [0, 25, 50, 75, 100], ...
                               'ValueChangedFcn', @(src, event) updateEndEffector(src));

    % End-effector control panel with 3x2 button layout for spatial control
    arrow_pad = uipanel(control_layout, 'Title', 'End-Effector Movement');
    pad_layout = uigridlayout(arrow_pad, [3, 2]);
    step_size = params.stepSize;

    % Buttons for end-effector movement
    uibutton(pad_layout, 'Text', '+Y', 'ButtonPushedFcn', @(src, event) queueMovement([0, step_size, 0]));
    uibutton(pad_layout, 'Text', '-Y', 'ButtonPushedFcn', @(src, event) queueMovement([0, -step_size, 0]));
    uibutton(pad_layout, 'Text', '-X', 'ButtonPushedFcn', @(src, event) queueMovement([-step_size, 0, 0]));
    uibutton(pad_layout, 'Text', '+X', 'ButtonPushedFcn', @(src, event) queueMovement([step_size, 0, 0]));
    uibutton(pad_layout, 'Text', '+Z', 'ButtonPushedFcn', @(src, event) queueMovement([0, 0, step_size]));
    uibutton(pad_layout, 'Text', '-Z', 'ButtonPushedFcn', @(src, event) queueMovement([0, 0, -step_size]));

    % Update end-effector based on slider
    function updateEndEffector(src)
        % Set the position for both prismatic joints
        end_effector_pos = src.Value; % Slider value (0 to 100)
        grip_dist = 0.025 * (end_effector_pos / 100); % Distance (0 to 0.025)
        
        % Update prismatic joint values
        joint_angles(7) = grip_dist;
        joint_angles(8) = grip_dist;
        
        % Update the robot display
        update_joint_angles(joint_angles);
    end

    % Queue end-effector movements
    function queueMovement(delta)
        moveQueue{end+1} = delta;
        if ~busy
            processQueue();
        end
    end

    % Process movement queue
    function processQueue()
        if isempty(moveQueue)
            busy = false;
            return;
        end

        busy = true;
        delta = moveQueue{1};
        moveQueue(1) = []; % Remove the movement from the queue
        move_end_effector(delta);

        % Process the queue again if it’s not empty
        busy = false;
        if ~isempty(moveQueue)
            processQueue();
        end
    end

    % Move the end effector incrementally
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

    % Update sliders and displays with the latest joint angles
    function update_sliders()
        for i = 1:6
            if ~sliderLock(i)
                slider_value = mod(joint_angles(i) + pi, 2 * pi) - pi;
                sliders(i).Value = slider_value;
                slider_displays(i).Value = slider_value;
            end
        end
    end

    % Update joint angles with interpolation
    function update_joint_angles(new_angles)
        num_steps = 50;
        t = linspace(0, 1, num_steps);
        joint_trajectory = zeros(num_steps, 8);

        for i = 1:8
            joint_trajectory(:, i) = spline([0 1], [last_angles(i), new_angles(i)], t);
        end

        for step = 1:num_steps
            current_angles = joint_trajectory(step, :);
            show(robot, current_angles, 'Parent', ax, 'Visuals', 'on', 'Frames', 'off', 'FastUpdate', true, 'PreservePlot', false);
            joint_angles = current_angles;
            update_sliders();
            pause(0.01);
        end

        last_angles = new_angles;
    end

    % Callback for slider updates
    function updateJointAngle(src, joint_index)
        sliderLock(joint_index) = true;
        new_angles = joint_angles;
        new_angles(joint_index) = src.Value;
        update_joint_angles(new_angles);
    end

    % Close program function
    function closeProgram()
        stopProgram = true;
        delete(ui_fig);
    end
end

% Ground plane creation for visualization
function create_ground_plane(x_range, y_range, color, alpha, ax)
    patch(ax, 'XData', [x_range(1), x_range(2), x_range(2), x_range(1)], ...
          'YData', [y_range(1), y_range(1), y_range(2), y_range(2)], ...
          'ZData', [0, 0, 0, 0], ...
          'FaceColor', color, ...
          'FaceAlpha', alpha, ...
          'EdgeColor', 'none');
end

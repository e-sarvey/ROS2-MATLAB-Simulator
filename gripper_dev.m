% Load the base robot
clear; close all
robot = loadrobot('universalUR3', 'Gravity', [0, 0, -9.81]);

% end_effector = importrobot("robotiq_hande_description-humble-devel/urdf/robotiq_hande.urdf");
end_effector = load("robotiq_hande.mat").robotiq_hande;
addSubtree(robot,"tool0",end_effector)
showdetails(robot)

show(robot, Frames="off");
% Get the current configuration of the robot
config = homeConfiguration(robot);

% Set the position of the prismatic joints at indices 13 and 14
config(7).JointPosition = set_gripper(50); % Set this to desired displacement for hande_left_finger_joint
config(8).JointPosition = set_gripper(50); % Set this to desired displacement for hande_right_finger_joint
figure(2)
% Visualize the robot with the updated joint positions
show(robot, config, Frames="off");

function finger_disp = set_gripper(value)
% function to set gripper to a value 0 (Closed) to 100 (Open) just like the
% real one.
    if value > 100
        value = 100;
    elseif value < 0
        value = 100;
    end

    finger_disp = value*(0.025/100);
    
    
end
    




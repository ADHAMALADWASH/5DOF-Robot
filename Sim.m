% مسح الذاكرة والشاشة
clear; clc; close all;

% ==========================================
% 1. بناء مجسم الروبوت 5-DOF بلمسة هندسية واقعية
% ==========================================
L1 = 0.20; L2 = 0.40; L3 = 0.30; L4 = 0.15;
robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 5);

body1 = rigidBody('link1'); joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1, [0, pi/2, L1, 0], 'dh');
addVisual(body1, 'Cylinder', [0.05, L1], trvec2tform([0, L1/2, 0]) * eul2tform([0, 0, pi/2]));
body1.Joint = joint1; addBody(robot, body1, 'base');

body2 = rigidBody('link2'); joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2, [L2, 0, 0, 0], 'dh');
addVisual(body2, 'Cylinder', [0.035, L2], trvec2tform([-L2/2, 0, 0]) * eul2tform([0, pi/2, 0]));
body2.Joint = joint2; addBody(robot, body2, 'link1');

body3 = rigidBody('link3'); joint3 = rigidBodyJoint('joint3', 'revolute');
setFixedTransform(joint3, [L3, 0, 0, 0], 'dh');
addVisual(body3, 'Cylinder', [0.03, L3], trvec2tform([-L3/2, 0, 0]) * eul2tform([0, pi/2, 0]));
body3.Joint = joint3; addBody(robot, body3, 'link2');

body4 = rigidBody('link4'); joint4 = rigidBodyJoint('joint4', 'revolute');
setFixedTransform(joint4, [0, pi/2, 0, 0], 'dh');
addVisual(body4, 'Sphere', 0.04); 
body4.Joint = joint4; addBody(robot, body4, 'link3');

body5 = rigidBody('link5'); joint5 = rigidBodyJoint('joint5', 'revolute');
setFixedTransform(joint5, [0, 0, L4, 0], 'dh');
addVisual(body5, 'Cylinder', [0.015, L4], trvec2tform([0, 0, -L4/2]));
body5.Joint = joint5; addBody(robot, body5, 'link4');

qHome = homeConfiguration(robot);

% ==========================================
% 2. إنشاء بيئة العمل (توسيع المسافة بين العائق والذراع)
% ==========================================
obstacle = collisionCylinder(0.06, 0.5); 
obstacle.Pose = trvec2tform([0.35, 0.0, 0.25]); % إبعاد العائق للأمام
env = {obstacle};

% ==========================================
% 3. الكينماتيكا العكسية (نقاط البداية والهدف بعيدة جداً عن العائق)
% ==========================================
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1, 1, 0, 1, 1, 1]; 
initialGuess = qHome;

% نقطة البداية (إرجاعها للخلف وتوسيعها لليمين)
startPose = trvec2tform([0.15, -0.45, 0.2]) * eul2tform([0, pi/2, 0]);
[qStart, ~] = ik('link5', startPose, weights, initialGuess);

% نقطة الهدف (إرجاعها للخلف وتوسيعها لليسار)
goalPose = trvec2tform([0.15, 0.45, 0.2]) * eul2tform([0, -pi/4, 0]); 
[qGoal, ~] = ik('link5', goalPose, weights, initialGuess);

% ==========================================
% 4. تخطيط المسار الديناميكي (Obstacle Avoidance)
% ==========================================
planner = manipulatorRRT(robot, env);
planner.MaxConnectionDistance = 0.25;
planner.ValidationDistance = 0.01;

disp('Planning safe path... Please wait.');
rng(1); 
path = plan(planner, qStart', qGoal');

if isempty(path)
    error('Failed to find a collision-free path.');
end

% تنعيم المسار
numSamples = 120;
timeVector = linspace(0, 4, size(path, 1));
[qTrajectory, ~, ~] = minjerkpolytraj(path', timeVector, numSamples);

% ==========================================
% 5. محاكاة الأنيميشن (Aesthetic Dark Theme)
% ==========================================
fig = figure('Name', 'Pro Robotics Simulation', 'NumberTitle', 'off', 'Position', [100 100 900 700]);
set(fig, 'Color', [0.1 0.12 0.16]); 

ax = axes('Parent', fig);
set(ax, 'Color', [0.15 0.17 0.21], 'XColor', '#8b9bb4', 'YColor', '#8b9bb4', 'ZColor', '#8b9bb4', 'GridColor', '#8b9bb4');
grid on; hold on;

xlim([-0.2 0.7]); ylim([-0.6 0.6]); zlim([0 0.8]);
view(140, 20); 
title('5-DOF Manipulator: Smart Obstacle Avoidance', 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');

camlight('right');
material('shiny'); 

[~, objPatches] = show(obstacle, 'Parent', ax);
objPatches.FaceColor = [0.8 0.2 0.3]; 
objPatches.FaceAlpha = 0.85; 
objPatches.EdgeColor = 'none';

eePath = plot3(0,0,0, '-g', 'LineWidth', 2.5); 
eePoses = zeros(3, numSamples);

for i = 1:numSamples
    qCurrent = qTrajectory(:, i);
    
    show(robot, qCurrent, 'PreservePlot', false, 'Frames', 'off', 'Parent', ax);
    
    eeTform = getTransform(robot, qCurrent, 'link5');
    eePoses(:, i) = eeTform(1:3, 4);
    
    set(eePath, 'XData', eePoses(1, 1:i), ...
                'YData', eePoses(2, 1:i), ...
                'ZData', eePoses(3, 1:i));
    drawnow;
end
disp('Animation Complete! Ready for screen recording.');
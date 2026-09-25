% Clear workspace and command window
clear; clc;

% ==========================================
% 1. Define 5-DOF Robot Link Lengths (in meters)
% ==========================================
L1 = 0.20; % Base joint height
L2 = 0.40; % Shoulder link length
L3 = 0.30; % Elbow link length
L4 = 0.15; % Wrist and tool length

% ==========================================
% 2. Define the Target Pose
% ==========================================
x = 0.3;      % X-coordinate
y = 0.2;      % Y-coordinate
z = 0.3;      % Height (Z-coordinate)

% End-effector orientation angles
phi_deg = -45; % Tool pitch angle (downward)
psi_deg = 0;   % Tool roll angle

% Convert required angles to radians for calculations
phi = deg2rad(phi_deg);
psi = deg2rad(psi_deg);

% ==========================================
% 3. Analytical Inverse Kinematics Calculations
% ==========================================

% 1. Calculate Base Yaw angle
theta1 = atan2(y, x);

% 2. Calculate Wrist Center coordinates in 2D plane
r = sqrt(x^2 + y^2);
rw = r - L4 * cos(phi);
zw = z - L4 * sin(phi);

% 3. Verify if the target is reachable (Workspace Check)
D = (rw^2 + (zw - L1)^2 - L2^2 - L3^2) / (2 * L2 * L3);

if D > 1 || D < -1
    error('Target is Out of Workspace. Please change (x, y, z) coordinates.');
end

% 4. Calculate Elbow Pitch angle - Choosing the "Elbow Up" solution
theta3 = atan2(-sqrt(1 - D^2), D); 

% 5. Calculate Shoulder Pitch angle
theta2 = atan2(zw - L1, rw) - atan2(L3 * sin(theta3), L2 + L3 * cos(theta3));

% 6. Calculate Wrist Pitch angle
theta4 = phi - (theta2 + theta3);

% 7. Calculate Wrist Roll angle
theta5 = psi;

% ==========================================
% 4. Display Results (Angles for Servos)
% ==========================================
% Convert final results from radians to degrees
angles_deg = rad2deg([theta1, theta2, theta3, theta4, theta5]);

fprintf('--- Required Servo Motor Angles to Reach Target ---\n');
fprintf('Motor 1 (Base Yaw)      : %7.2f degrees\n', angles_deg(1));
fprintf('Motor 2 (Shoulder Pitch): %7.2f degrees\n', angles_deg(2));
fprintf('Motor 3 (Elbow Pitch)   : %7.2f degrees\n', angles_deg(3));
fprintf('Motor 4 (Wrist Pitch)   : %7.2f degrees\n', angles_deg(4));
fprintf('Motor 5 (Wrist Roll)    : %7.2f degrees\n', angles_deg(5));
fprintf('---------------------------------------------------\n');

function [F, M, trpy, drpy] = nlcontroller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% Extract state variables and desired variables from input
r_des = qd{qn}.pos_des;
rdot_des = qd{qn}.vel_des;
rddot_des = qd{qn}.acc_des;
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;

r_st = qd{qn}.pos;
rdot_st = qd{qn}.vel;
euler_st = qd{qn}.euler;
omega_st = qd{qn}.omega;

% 
g = params.grav;
m = params.mass;
I = params.I;

% Define control parameters
kprx = 5.55;
kpry = 5.55;
kprz = 6;
kdrx = 5.3;
kdry = 5.3;
kdrz = 5.5;

kdr = [kdrx 0 0; 0 kdry 0; 0 0 kdrz];
kpr = [kprx 0 0; 0 kpry 0; 0 0 kprz];
kp_phi = 2700;
kp_th = 2700;
kp_psi = 2000;
kd_phi = 200;
kd_th = 200;
kd_psi = 200;

kpw = [kp_phi 0 0; 0 kp_th 0; 0 0 kp_psi];
kdw = [kd_phi 0 0; 0 kd_th 0; 0 0 kd_psi];

rddot_com = rddot_des - kdr*(rdot_st - rdot_des) - kpr*(r_st - r_des);

F_des = m*rddot_com + [0;0;m*g];

u    = zeros(4,1); % control input u

R_st = eulzxy2rotmat(euler_st);
b3 = R_st*[0;0;1];
u(1) = b3'*F_des;

b3_des = F_des/norm(F_des);
b2_des = cross(b3_des,[0;0;1])/norm(cross(b3_des,[0;0;1]));
b1_des = cross(b2_des,b3_des);

R_des(:,1) = b1_des;
R_des(:,2) = b2_des;
R_des(:,3) = b3_des;

err = R_des'*R_st - R_st'*R_des;

err_R = 0.5*veemap(err);

p_des = 0;
q_des = 0;
r_des = yawdot_des;

omega_des = [p_des;q_des;r_des];

err_w = omega_st - omega_des;

u(2:4) = I*(-kpw*err_R' - kdw*err_w);

F = F_des(3);

M = u(2:4);

euler = rotmat2eulzxy(R_des);
phi_des = euler(1);
theta_des = euler(2);
psi_des = euler(3);

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end


function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
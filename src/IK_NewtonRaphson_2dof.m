clear all; clc; close all; 

% Numerical calculation parameters
max_iter = 100;
max_error = 0.01;

% Enter the lengths of the links
l1 = input ("Enter the lenght of the first link:  ");
l2 = input("Enter the length of the second link:  ");

% Enter the end-effector position
xe = input(" Enter the end-effector x value:  ");
ye = input ('Enter the end-effector y value:  ');

% Initialization
ii = 1; 
error(ii)= 10; 

% Enter the initial guesses for the angles
theta1(ii)= pi* input('Enter the intial value for theta 1 in degrees:  ')/180; 
theta2(ii) = pi* input('Enter the intial value for theta 2 in degrees:  ')/180;

%Applying the Newton-Raphson Method

while (ii < max_iter) && (error(ii) > max_error)
    % define the position of the endeffector in terms of the joint variables
    xpos = l1*cos(theta1(ii))+l2*cos(theta1(ii)+theta2(ii)); 
    ypos = l1*sin(theta1(ii))+l2*sin(theta1(ii)+theta2(ii));
    mat1 = [-l1*sin(theta1(ii))-l2*sin(theta1(ii)+theta2(ii))  -12*sin(theta1(ii)+theta2(ii));
            -l1*cos(theta1(ii))+l2*cos(theta1(ii)+theta2(ii))   12*cos(theta1(ii)+theta2(ii))];
    %Error Matrix
    mat2 = [xpos-xe ;ypos-ye]; 
    theta = [theta1(ii); theta2(ii)] - (inv(mat1) * mat2); 
    ii = ii +1;
    theta1(ii) = theta(1);
    theta2(ii) = theta(2); 
    error(ii) = norm(mat2)
end 

% Plots the results

figure ;
title('Inverse Kinematics solution for 2DOF Planar Arm')
subplot(3,1,1);
plot(theta1,'*-');
xlabel('iteration number');
ylabel('theta1');

subplot(3,1,2); 
plot(theta2,'*-');
xlabel('iteration number');
ylabel('theta2');

subplot(3,1,3); 
plot(error,'*-');
xlabel('iteration number');
ylabel('End-effector forward Kin Error');

% % Visual of the results
figure; 
plot(xe,ye, 'rx','MarkerFaceColor',[1 0 0]);
hold on;
line([0 l1*cos(theta1(end))],[0 l1*sin(theta1(end))], 'LineWidth',3);
line(l1*cos(theta1(end))+l2*cos(theta1(end)+theta2(end)) ,l1*sin(theta1(end))+l2*sin(theta1(end)+theta2(end)),'Linewidth',3)
plot(xe,ye,'go','MarkerSize',3,'MarkerFaceColor',[0 0 1]);
legend('Actual end- effector position','computed robot pose'); 


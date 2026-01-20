clear all; clc; close all; 

% Enter the lengths of the links
% l1 = input ("Enter the lenght of the first link:  ");
% l2 = input("Enter the length of the second link:  ");
a1 = 1;
a2 = 1;
% Enter the end-effector position
% xe = input(" Enter the end-effector x value:  ");
% ye = input ('Enter the end-effector y value:  ');
th1a = 0;
th2a = pi/2;

%Forward kinematic
mu_a = [a1*cos(th1a)+a2*cos(th1a +th2a); 
        a1*sin(th1a)+a2*sin(th1a +th2a)]; 
% Enter the initial guesses for the angles
% th1 = pi* input('Enter the intial value for theta 1 in degrees:  ')/180; 
% th2 = pi* input('Enter the intial value for theta 2 in degrees:  ')/180;
q = [pi/4;pi/3]; 
angles1 = [];
angles2 = [];
errorValues = [];

tic
%Newtons method begins here
for i = 1:50
    th1 = q(1);
    th2 = q(2);
    %Jacobian Matrix
    J = [-a1*sin(th1)-a2*sin(th1+th2),  -a2*sin(th1+th2);
         -a1*cos(th1)+a2*cos(th1+th2),  a2*cos(th1+th2)];
    %Estimated task-space position
    mu_e = [a1*cos(th1)+a2*cos(th1+th2); a1*sin(th1)+a2*sin(th1+th2)];
    %The error
    delta = mu_a - mu_e;
    %Tolerance check 
    if norm(delta)<1e-5
        break;
    end
    %New point 
    q = q + inv(J)*(delta) ;
    angles1(i) = q(1);
    angles2(i) = q(2); 
    errorValues(i) = norm(delta);
    %Plots
    % plot ([0 a1*cos(th1) mu_e(1)],[0 a1*sin(th1) mu_e(2)],'b-o')
    % hold on
    % plot(mu_a(1), mu_a(2), "r*","MarkerSize",10)
    % plot(0,0,'ks','MarkerSize',10);
    % axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2)])
    % grid on
end
i
toc

figure ;
title('Inverse Kinematics solution for 2DOF Planar Arm')
subplot(3,1,1);
plot(angles1,'*-');
xlabel('iteration number');
ylabel('theta1');
hold on
subplot(3,1,2); 
plot(angles2,'*-');
xlabel('iteration number');
ylabel('theta2');
hold on
subplot(3,1,3); 
plot(errorValues,'*-');
xlabel('iteration number');
ylabel('End-effector forward Kin Error');
hold on

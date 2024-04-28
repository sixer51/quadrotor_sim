clc; clear; close all

addpath('./src');

% QUADROTOR

g = 9.81;  % The gravitational acceleration [m/s^2]
l = 0.2;  % Distance from the center of mass to each rotor [m]
m = 0.5;  % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48];  % Mass moment of inertia [kg m^2]
mu = 3.0;  % Maximum thrust of each rotor [N]
sigma = 0.01;  % The proportionality constant relating thrust to torque [m]

quad = quadrotor(g, l, m, diag(I), mu, sigma);

% INTRUDER
% circle path
% path = @(t) [cos(t); sin(t); 2];

% straight line path
% 0.1+0.2*t
% path = @(t) [-5+0.2*t; cos(t); 3];
% path = @(t) [1; 1; t];

% path = @(t) [-2+0.2*t; -5+0.1*t; 2+0.2*t];

% staright, horizontal movement
path = @(t) [-5+0.5*t; 1; 3];

dist = struct("r", @(t,z)0.1*[sin(t); sin(2*t); sin(4*t)],...
    "n", @(t,z) 0.1*[0.1; 0.01; 0.1]);
 
intruder = uav(path, dist);

% CONTROLLER
ctrl = lqr_controller(quad);

% SIMULATION

sim = simulator(quad, ctrl, intruder);
sim.simtime = [0 10];
sim.timestep = 0.01;
sim.epsilon = 0.1;

z0 = zeros(12,1);

[t,z,u,d,y] = sim.simulate(z0);

% ANIMATION
sim.animate(t, z, y);

% Plotting quadrotor states
% figure(2)
% for i=1:4
%     ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
%                 'Xlim',[t(1), t(end)],...
%                 'TickLabelInterpreter','LaTeX','FontSize',14);
%     xlabel('t','Interpreter','LaTeX','FontSize',14);        
% end
% 
% 
% plot(ax(1), t,z(:,1:3), 'LineWidth', 1.5);
% legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
% xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(3), t, z(:,4:6), 'LineWidth', 1.5);
% legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(3), '\boldmath$\alpha$','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(2), t, z(:,7:9), 'LineWidth', 1.5);
% legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(4), t, z(:,10:12), 'LineWidth', 1.5);
% legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(4), '\boldmath$\omega$','Interpreter','LaTeX','FontSize',14);

% Plotting quadrotor states
figure()
for i=1:6
    ax(i) = subplot(3,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                'Xlim',[t(1), t(end)],...
                'TickLabelInterpreter','LaTeX','FontSize',14);
    xlabel('t','Interpreter','LaTeX','FontSize',14);        
end


plot(ax(1), t,[z(:,1), y(:,1)], 'LineWidth', 1.5);
legend(ax(1), {'$x_{quad}$', '$x_{uav}$'},... 
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);

plot(ax(3), t, [z(:,2), y(:,2)], 'LineWidth', 1.5);
legend(ax(3), {'$y_{quad}$', '$y_{uav}$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(2), '${\bf y}$','Interpreter','LaTeX','FontSize',14);

plot(ax(5), t, [z(:,3), y(:,3)], 'LineWidth', 1.5);
legend(ax(5), {'$z_{quad}$', '$z_{uav}$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(2), '$${\bf z}$','Interpreter','LaTeX','FontSize',14);

plot(ax(2), t,z(:,1)- y(:,1), 'LineWidth', 1.5);
title(ax(2), '${\bf x error}$','Interpreter','LaTeX','FontSize',14);
xlabel(ax(2), 't','Interpreter','LaTeX','FontSize',14);

plot(ax(4), t, z(:,2)- y(:,2), 'LineWidth', 1.5);
title(ax(4), '${\bf y error}$','Interpreter','LaTeX','FontSize',14);
 
plot(ax(6), t, z(:,3)- y(:,3), 'LineWidth', 1.5);
title(ax(6), '$${\bf z error}$','Interpreter','LaTeX','FontSize',14);
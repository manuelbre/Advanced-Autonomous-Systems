function [] = particleSwarmOptimization (vel_fis_file, ang_fis_file)
%% DESCRIPTION
%
% Simple Fuzzy control of a mobile robot. The goal is to move the robot ...
% from a start position S to a target positon T. This script is used to ...
% to demonstrate a simple, non-optimized version of fuzzy control.
%

%% INITIALIZATION

% Preprocessing
clc; clear all; close all;
% Add that folder plus all subfolders to the path.
dirs = fileparts(which(mfilename)); 
addpath(genpath(dirs));
% Turnoff fis eval warning
warning('off', 'Fuzzy:evalfis:InputOutOfRange')


% % % % % % % % % Params % % % % % % % % % % % % 

% Number of generations
n_g = 2;
% Number of Particles
n_p = 10;

% Size of Map
map_range = 50;

% Virtual target range
d_virt_T_range = [0; 50];
% Particle velocity range
part_vel_range = [-1; 1];
% Particle personal best coefficient (cognition coefficient)
c_p = 0.7;
% Particle global best coefficient (social coefficient)
c_g = 1;

% End time [s]
t_end = 300;

% Timestep [s]
dt = 0.1;

% Disp flag
disp_sim = false;

% % % % % % % % % Params % % % % % % % % % % % % 

% Fis files
default_vel_fis_file = 'MTRN4010_vel.fis';
default_ang_fis_file = 'MTRN4010_ang.fis';
if ~exist('vel_fis_file','var'), vel_fis_file = default_vel_fis_file; end
if ~exist('ang_fis_file','var'), ang_fis_file = default_ang_fis_file; end

%% FUNCTIONALITY

% Init particles
particles = Particles(n_p, c_p, c_g, d_virt_T_range, part_vel_range);

disp(sprintf("Start particle swarm optimization with %d particles and %d generations.\n", n_p, n_g));
disp("-------------------------------------------------")
disp("-------------------------------------------------")
pause(0.001)

for gen = 1:n_g    
    for p_idx = 1:n_p
        d_virt_T_0 = particles.getPosition(p_idx);
        
        % Simulate robot with fuzzy control
        [X, X_T] = fuzzyControl (map_range, d_virt_T_0, t_end, ...
            dt, disp_sim, vel_fis_file, ang_fis_file);
        
        % Evaluate fitness of particle and update it
        fitness = compFitness(X, X_T);
        particles.setFitness(p_idx, fitness)
    end
    
    particles.updatePersonalBests();
    particles.updateGlobalBest();
    particles.updateVelPos();
    
    disp(sprintf("Global best fitness of generation %d is %0.5f with virt target distance of %0.5f\n", gen, ...
                       particles.globalBest.val, particles.globalBest.pos));
    pause(0.001)
end

% Display end result
disp("-------------------------------------------------")
disp("-------------------------------------------------")

disp(sprintf("Best particle has virtual target distance %0.5f with a fitness of %0.5f\n", ...
                    particles.globalBest.pos, particles.globalBest.val))

disp_sim = true;
[X, X_T] = fuzzyControl (map_range, particles.globalBest.pos, t_end, ...
            dt, disp_sim, vel_fis_file, ang_fis_file);

end

function [fitness] = compFitness(X_end, X_T)
%% DESCRIPTION
% Compute fitness of end state by using error to target state.
%
% INPUT:
%       - X_end (3 x 1): End robot state
%       - X_T (3 x 1): Target robot state
%
% OUTPUT:
%       - fitness (1 x 1): Fitness value

%% FUNCTIONALLITY

% Convert rad to deg (according to exercise description)
X_end_theta = rad2deg(X_end(3));
X_T_theta = rad2deg(X_T(3));

fitness = 1 ./ sqrt( (X_end(1) - X_T(1)).^2 + (X_end(1) - X_T(1)).^2 + ...
               (X_end_theta - X_T_theta).^2 );

end

% function h = create
%     figure(2)
%     surf(particles.positions, particles.velocities, particles.personalBests.val)
%     xlabel('Virtual Target Distance [m]')
%     ylabel('Particle Velocity [-]')
%     zlabel('Particle Fitness [-]')
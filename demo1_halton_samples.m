% demo1_halton_samples.m
% Task 1: Generate Halton samples in the 3D C-space (x, y, theta)
clear; clc; close all;

fprintf('=== Demo 1: Generate Halton Samples in 3D C-Space ===\n');

% Warehouse bounds
xBounds = [0, 20];
yBounds = [0, 15];
num_samples = 1500;

% Obtain Halton uniformly distributed pseudo-random sequence
samples = generate_halton_samples_3d(num_samples, xBounds, yBounds);

% Visualize
figure('Name', 'Task 1: Halton Samples', 'Color', 'w', 'Position', [100, 100, 800, 600]);
scatter3(samples(:,1), samples(:,2), samples(:,3), 15, 'b', 'filled');
xlabel('X (m)', 'FontWeight', 'bold'); 
ylabel('Y (m)', 'FontWeight', 'bold'); 
zlabel('Theta (rad)', 'FontWeight', 'bold');
title(sprintf('%d Halton Sequence Samples in 3D (x, y, \theta)', num_samples), 'FontSize', 14);
grid on;
view(-45, 30);

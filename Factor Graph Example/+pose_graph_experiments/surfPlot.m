clear all;
close all;
clc;

import pose_graph_experiments.*;

% Parameters for generate plots
os = "win";
weekNum = 1;

% Define the range and step for Omega scales
omegaRScaleArray = 0.1:0.1:1.9;
omegaQScaleArray = 0.1:0.1:1.9;

% Plots
getAllSurfPlots(os, weekNum, omegaRScaleArray, omegaQScaleArray);

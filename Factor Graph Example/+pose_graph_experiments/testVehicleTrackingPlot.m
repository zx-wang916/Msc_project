close all;
clear all;
clc;

import g2o.core.*;
import pose_graph_experiments.*;
import odometry_model_answer.*;

% Number of steps per episode
numberOfTimeSteps = 20;

% Number of episodes
numberOfEpisodes = 100;

% If set to false, we test proposition 3, which initializes the graph at the
% ground truth value, and does not optimize. If set to true, we test
% proposition 4, which is the distribution after optimizing with noisy
% measurements
testProposition4 = false;

% Parameters to change the frequency of measurement updates
numObs = 100;
obsPeriod = 1;

% Number of landmarks and layout
numberOfLandmarks = 200;
extent = 300;

% Populate the landmark locations
landmarks = (rand([2 numberOfLandmarks]) - 0.5) * extent;

% Define the range and step for Omega scales
omegaRScaleArray = 0.1:0.1:1.9;
omegaQScaleArray = 0.1:0.1:1.9;

[RM, QM] = meshgrid(omegaRScaleArray, omegaQScaleArray);

% Create a matrix to store C values
C_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));

% Create matrices to store meanChi2 and covChi2 values
meanChi2_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));
covChi2_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));

% Loop over all possible Omega values
parfor i = 1:numel(RM)
        
    % Initialize chi2Store for each episode
    chi2SumStore = zeros(numberOfEpisodes, 1);
    dimXStore = zeros(numberOfEpisodes, 1);
    dimZStore = zeros(numberOfEpisodes, 1);

%     edgeStore = cell(numberOfEpisodes, 1);

    % Get chi2Sum and chi2 values, along with the edges in graph for each
    % running episode
    fprintf('OmegaRScale: %.2f, OmegaQScale: %.2f\n', ...
        RM(i), QM(i))

    for r = 1 : numberOfEpisodes
        [chi2SumStore(r), ~, ~, dimXStore(r), dimZStore(r)] = ...
            runVehicleTrackingExample(numberOfTimeSteps, ...
            RM(i), QM(i), testProposition4, landmarks);
    end

    dimX = mean(dimXStore);
    dimZ = mean(dimZStore);

    % Calculate meanChi2, covChi2
    meanChi2 = mean(chi2SumStore);
    covChi2 = cov(chi2SumStore);

    % Compute the number of degrees of freedom
    if (testProposition4 == true)
        N = dimZ - dimX;
    else
        N = dimZ;
    end

    % Store C, meanChi2, and covChi2 in matrices
    C_store(i) = abs(log(meanChi2/N)) + abs(log(covChi2/(2*N)));
    meanChi2_store(i) = meanChi2;
    covChi2_store(i) = covChi2;
end

%delete(gcp('nocreate')); % stop the parallel pool

%%
% Parameters to determine path to store
os = "win";
weekNum = 1;
systemName = "vehicle";

% Path to store plots
if os == "mac"
    basePath = "~/Desktop/week" + weekNum + "/";
else
    basePath = "D:\University\UCL\project\week" + weekNum + "\";
end

% Create folder if not exist and display log
if ~exist(basePath, 'dir')
    mkdir(basePath);
    disp('Folder created successfully.');
else
    disp('Folder already exists.');
end

% Build the file names
C_name = "C_" + systemName;
Mean_name = "meanChi2_" + systemName;
Cov_name = "covChi2_" + systemName;

if testProposition4 == true
    C_name = C_name + "_prop4";
    Mean_name = Mean_name + "_prop4";
    Cov_name = Cov_name + "_prop4";
end

% Plotting
figure(1)
imagesc(omegaRScaleArray, omegaQScaleArray, C_store);
colorbar; % adds a colorbar, which indicates the scale of C values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Consistency Measurement (C) for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y
saveas(gcf, basePath + C_name + ".png")
writematrix(C_store, basePath + C_name + '.csv')

% Plot the meanChi2 and covChi2 values
figure(2)
imagesc(omegaRScaleArray, omegaQScaleArray, meanChi2_store);
colorbar; % adds a colorbar, which indicates the scale of meanChi2 values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Mean Chi2 for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y
saveas(gcf, basePath + Mean_name + ".png")
writematrix(meanChi2_store, basePath + Mean_name + '.csv')

% Plot the meanChi2 and covChi2 values
figure(3)
imagesc(omegaRScaleArray, omegaQScaleArray, covChi2_store);
colorbar; % adds a colorbar, which indicates the scale of covChi2 values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Covariance Chi2 for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y
saveas(gcf, basePath + Cov_name + ".png")
writematrix(covChi2_store, basePath + Cov_name + '.csv')

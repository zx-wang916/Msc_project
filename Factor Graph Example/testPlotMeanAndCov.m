close all;
clear all;
clc;

import g2o.core.*;
import two_d_tracking_model_answer.*;


% Number of steps per episode
numberOfTimeSteps = 100;

% Number of episodes
numberOfEpisodes = 2000;

% If set to false, we test proposition 3, which initializes the graph at the
% ground truth value, and does not optimize. If set to true, we test
% proposition 4, which is the distribution after optimizing with noisy
% measurements
testProposition4 = false;

% Define the range and step for Omega scales
omegaRScaleArray = 1.0:0.1:1.9;
omegaQScaleArray = 1.0:0.1:1.9;

% Create a matrix to store C values
C_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));

% Create matrices to store meanChi2 and covChi2 values
meanChi2_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));
covChi2_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));

% Start a parallel pool
parpool;

% Loop over all possible Omega values
for i = 1:length(omegaRScaleArray)
    for j = 1:length(omegaQScaleArray)
        
        % Initialize chi2Store for each episode
        chi2Store = zeros(numberOfEpisodes, 2 * numberOfTimeSteps - 1);
        chi2SumStore = zeros(numberOfEpisodes, 1);
        edgeStore = cell(numberOfEpisodes, 1);

        % Get chi2Sum and chi2 values, along with the edges in graph for each
        % running episode
        parfor r = 1 : numberOfEpisodes
            fprintf('OmegaRScale: %.2f, OmegaQSclae: %.2f, Episode: %03d\n',...
                omegaRScaleArray(i), omegaQScaleArray(j),r)
            [edges, chi2SumStore(r), chi2Store(r, :)] = ...
                runLinearExample(numberOfTimeSteps, ...
                omegaRScaleArray(i), omegaQScaleArray(j), testProposition4);
            % Store the edges in a cell array
            edgeStore{r} = edges;
        end

        % Compute the number of dimensions
        % We can compute it using edges from any episode, as they are the same for
        % all episodes.
        dimZ = 0; % True value of n (n_x + n_z)
        for k = 1 : length(edgeStore{1})
            dimZ = dimZ + edgeStore{1}{k}.dimension();
        end

        % Calculate S, meanChi2, covChi2
        timestepMean = mean(chi2Store,1);
        difference = chi2Store - timestepMean;
        squaredDifference = difference .^ 2;
        totalSum = sum(squaredDifference(:));
        S = totalSum / (numberOfTimeSteps * (numberOfEpisodes - 1));
        meanChi2 = mean(chi2SumStore);
        covChi2 = cov(chi2SumStore);

        % Store C, meanChi2, and covChi2 in matrices
        C_store(i,j) = abs(log(meanChi2/dimZ)) + abs(log(S/2*dimZ));
        meanChi2_store(i,j) = meanChi2;
        covChi2_store(i,j) = covChi2;
    end
end

delete(gcp('nocreate')); % stop the parallel pool

%%
% Plotting
figure(1)
imagesc(omegaRScaleArray, omegaQScaleArray, C_store);
colorbar; % adds a colorbar, which indicates the scale of C values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Consistency Measurement (C) for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y

% Plot the meanChi2 and covChi2 values
figure(2)
imagesc(omegaRScaleArray, omegaQScaleArray, meanChi2_store);
colorbar; % adds a colorbar, which indicates the scale of meanChi2 values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Mean Chi2 for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y

% Plot the meanChi2 and covChi2 values
figure(3)
imagesc(omegaRScaleArray, omegaQScaleArray, covChi2_store);
colorbar; % adds a colorbar, which indicates the scale of covChi2 values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Covariance Chi2 for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y
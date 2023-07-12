close all;
clear all;
clc;

import g2o.core.*;
import pose_graph_experiments.*;
import two_d_tracking_model_answer.*;


% Number of steps per episode
numberOfTimeSteps = 100;

% Number of episodes
numberOfEpisodes = 1000;

% If set to false, we test proposition 3, which initializes the graph at the
% ground truth value, and does not optimize. If set to true, we test
% proposition 4, which is the distribution after optimizing with noisy
% measurements
testProposition4 = true;

% Parameters to change the frequency of measurement updates
numObs = 20;
obsPeriod = 2;

% Define the range and step for Omega scales
omegaRScaleArray = 0.1:0.1:1.9;
omegaQScaleArray = 0.1:0.1:1.9;

[RM, QM] = meshgrid(omegaRScaleArray, omegaQScaleArray);

% Create a matrix to store C values
C_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));

% Create matrices to store meanChi2 and covChi2 values
meanChi2_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));
covChi2_store = zeros(length(omegaRScaleArray), length(omegaQScaleArray));

% Start a parallel pool
%parpool;

% Compute the number of edges
numberOfEdges = 2 * numObs - 1 + ...
    floor((numberOfTimeSteps - numObs) / obsPeriod) + ...
    numberOfTimeSteps - numObs;

% Loop over all possible Omega values
parfor i = 1:numel(RM)
        
    % Initialize chi2Store for each episode
    chi2Store = zeros(numberOfEpisodes, numberOfEdges);
    chi2SumStore = zeros(numberOfEpisodes, 1);
%     edgeStore = cell(numberOfEpisodes, 1);

    % Get chi2Sum and chi2 values, along with the edges in graph for each
    % running episode
    fprintf('OmegaRScale: %.2f, OmegaQScale: %.2f\n', ...
        RM(i), QM(i))

    % First run retrieves the graph dimensions
    [chi2SumStore(1), chi2Store(1, :), ~, dimX, dimZ] = ...
        runLinearExample(numberOfTimeSteps, ...
        RM(i), QM(i), testProposition4, ...
        numObs, obsPeriod);

    for r = 2 : numberOfEpisodes
        [chi2SumStore(r), chi2Store(r, :)] = ...
            runLinearExample(numberOfTimeSteps, ...
            RM(i), QM(i), testProposition4, ...
            numObs, obsPeriod);
    end

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
% Plotting
figure(1)
imagesc(omegaRScaleArray, omegaQScaleArray, C_store);
colorbar; % adds a colorbar, which indicates the scale of C values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Consistency Measurement (C) for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y
saveas(gcf, "~/Desktop/week9/C_measure_rate_20-2_prop4.png")
writematrix(C_store, '~/Desktop/week9/C_measure_rate_20-2_prop4.csv')

% Plot the meanChi2 and covChi2 values
figure(2)
imagesc(omegaRScaleArray, omegaQScaleArray, meanChi2_store);
colorbar; % adds a colorbar, which indicates the scale of meanChi2 values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Mean Chi2 for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y
saveas(gcf, "~/Desktop/week9/meanChi2_measure_rate_20-2_prop4.png")
writematrix(meanChi2_store, '~/Desktop/week9/meanChi2_measure_rate_20-2_prop4.csv')

% Plot the meanChi2 and covChi2 values
figure(3)
imagesc(omegaRScaleArray, omegaQScaleArray, covChi2_store);
colorbar; % adds a colorbar, which indicates the scale of covChi2 values
xlabel('omegaRScale');
ylabel('omegaQScale');
title('Covariance Chi2 for different Omega values');
axis xy; % flips the axis so omegaRScale is x and omegaQScale is y
saveas(gcf, "~/Desktop/week9/covChi2_measure_rate_20-2_prop4.png")
writematrix(covChi2_store, '~/Desktop/week9/covChi2_measure_rate_20-2_prop4.csv')

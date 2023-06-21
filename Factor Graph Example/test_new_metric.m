close all;
clear all;
clc;

import g2o.core.*;
import two_d_tracking_model_answer.*;


% Number of steps per episode
numberOfTimeSteps = 100;

% Number of episodes
numberOfEpisodes = 20;

% Omega Scales
omegaRScale = 1;
omegaQScale = 1;

% If set to false, we test proposition 3, which initializes the graph at the
% ground truth value, and does not optimize. If set to true, we test
% proposition 4, which is the distribution after optimizing with noisy
% measurements
testProposition4 = false;

chi2Store = zeros(numberOfEpisodes, 2 * numberOfTimeSteps - 1);
chi2SumStore = zeros(numberOfEpisodes, 1);
edgeStore = cell(numberOfEpisodes, 1);

% Get chi2Sum and chi2 values, along with the edges in graph for each
% running episode
parfor r = 1 : numberOfEpisodes
    fprintf('%03d\n', r)
    [edges, chi2SumStore(r), chi2Store(r, :)] = ...
        runLinearExample(numberOfTimeSteps, ...
        omegaRScale, omegaQScale, testProposition4);
    % Store the edges in a cell array
    edgeStore{r} = edges;
end

% Compute the number of dimensions
% We can compute it using edges from any episode, as they are the same for
% all episodes.
dimZ = 0; % True value of n (n_x + n_z)
for i = 1 : length(edgeStore{1})
    dimZ = dimZ + edgeStore{1}{i}.dimension();
end

% TimestepMean calculates the average chi2 value for a specific edge in the
% graph for all run. Not the average chi2 for all edges in the graph in a
% single run. 
timestepMean = mean(chi2Store,1);

% Compute S
difference = chi2Store - timestepMean;
squaredDifference = difference .^ 2;
totalSum = sum(squaredDifference(:));
S = totalSum / (numberOfTimeSteps * (numberOfEpisodes - 1));

% This figure works out and plots the mean and covarianace for the sum of
% the chi2 values over all edges. Propositions 3 and 4 are guaranteed to
% apply to ONLY these values.
figure(1)
plot(chi2SumStore)
meanChi2 = mean(chi2SumStore);
covChi2 = cov(chi2SumStore);
title(sprintf('Mean: %f; Covariance %f', meanChi2, covChi2))

% Compute the Consistency Measurement
C = abs(log(meanChi2/dimZ)) + abs(log(S/2*dimZ));


close all;
clear all;
clc;

import g2o.core.*;
import pose_graph_experiments.*;
import odometry_model_answer.*;

% Number of steps per episode
numberOfTimeSteps = 40;

% Number of episodes
numberOfEpisodes = 10000;

% Omega Scales
omegaRScale = 1;
omegaQScale = 1;

% If set to false, we test proposition 3, which initializes the graph at the
% ground truth value, and does not optimize. If set to true, we test
% proposition 4, which is the distribution after optimizing with noisy
% measurements
testProposition4 = true;

chi2Store = zeros(numberOfEpisodes, 2 * numberOfTimeSteps - 1);
chi2SumStore = zeros(numberOfEpisodes, 1);
% edgeStore = cell(numberOfEpisodes, 1);

R = diag([1.6148 1.7197]);
Q = diag([0.22683 0.11764 0.01808] .^2);

% First run retrieves the graph dimensions
[chi2SumStore(1), chi2Store(1, :), edges, dimX, dimZ] = ...
    runGPSExample(numberOfTimeSteps, ...
    omegaRScale, omegaQScale, testProposition4, ...
    R, Q);

% Get chi2Sum and chi2 values,
parfor r = 2 : numberOfEpisodes
    fprintf('%03d\n', r)
    [chi2SumStore(r), chi2Store(r, :)] = ...
        runGPSExample(numberOfTimeSteps, ...
        omegaRScale, omegaQScale, testProposition4, ...
        R, Q);
end

% Compute the number of degrees of freedom
if (testProposition4 == true)
    N = dimZ - dimX;
else
    N = dimZ;
end

% This figure works out and plots the mean and covarianace for the sum of
% the chi2 values over all edges. Propositions 3 and 4 are guaranteed to
% apply to ONLY these values.
figure(1)
plot(chi2SumStore)
meanChi2 = mean(chi2SumStore);
covChi2 = cov(chi2SumStore);
title(sprintf('Mean: %f; Covariance %f', meanChi2, covChi2))

% Compute the Consistency Measurement
C = abs(log(meanChi2/N)) + abs(log(covChi2/(2*N)));
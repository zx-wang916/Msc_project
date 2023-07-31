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

% Omega Scales
omegaRScale = 1;
omegaQScale = 1;

% Parameters to change the frequency of measurement updates
numObs = 100;
obsPeriod = 1;

% Number of subgraphs
numSubgraph = 4;

% If set to false, we test proposition 3, which initializes the graph at the
% ground truth value, and does not optimize. If set to true, we test
% proposition 4, which is the distribution after optimizing with noisy
% measurements
testProposition4 = true;

% Compute the number of edges
numberOfEdges = 2 * numObs - 1 + ...
    floor((numberOfTimeSteps - numObs) / obsPeriod) + ...
    numberOfTimeSteps - numObs;

if numSubgraph == 1
    chi2Store = zeros(numberOfEpisodes, numberOfEdges);
    chi2SumStore = zeros(numberOfEpisodes, 1);
else
    chi2Store = zeros(numberOfEpisodes, numSubgraph);
    chi2SumStore = zeros(numSubgraph, 1);
end

% Get chi2Sum and chi2 values, along with the edges in graph for each
% running episode

% First run retrieves the graph dimensions
[chi2SumStore(1), chi2Store(1, :), ~, dimX, dimZ] = ...
    runLinearExample(numberOfTimeSteps, ...
    omegaRScale, omegaQScale, testProposition4, ...
    numObs, obsPeriod, numSubgraph);

parfor r = 2 : numberOfEpisodes
    fprintf('%03d\n', r)
    [chi2SumStore(r), chi2Store(r, :)] = ...
        runLinearExample(numberOfTimeSteps, ...
        omegaRScale, omegaQScale, testProposition4, ...
        numObs, obsPeriod, numSubgraph);
end


% Compute the number of degrees of freedom
if (testProposition4 == true)
    N = dimZ - dimX;
else
    N = dimZ;
end

% TimestepMean calculates the average chi2 value for a specific edge in the
% graph for all run. Not the average chi2 for all edges in the graph in a
% single run. 
% timestepMean = mean(chi2Store,1);

% Compute S
% difference = chi2Store - timestepMean;
% squaredDifference = difference .^ 2;
% totalSum = sum(squaredDifference(:));
% S = totalSum / (numberOfTimeSteps * (numberOfEpisodes - 1));

if numSubgraph == 1
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
else
    % Compute the mean and covariance for each subgraph
    C = zeros(1, numSubgraph);
    for i = 1:numSubgraph
        figure(i)
        plot(chi2Store(:, i))
        meanChi2 = mean(chi2Store(:, i));
        covChi2 = cov(chi2Store(:, i));
        title(sprintf('Subgraph: %d, Mean: %f; Covariance %f', i, meanChi2, covChi2))
    
        % Compute the Consistency Measurement for each subgraph
        C(i) = abs(log(meanChi2/N)) + abs(log(covChi2/(2*N)));
    end
end



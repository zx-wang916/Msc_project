close all;
clear all;
clc;

% Parameters for plotting
os = "win";
weekNum = 12;
system_name = "linear";
saveResults = false;

% Number of steps per episode
numberOfTimeSteps = 100;

% Number of episodes
numberOfEpisodes = 1000;
    
% If set to false, we test proposition 3, which initialises the graph at the
% ground truth value, and does not optimise. If set to true, we test
% proposition 4, which is the distribution after optimising with noisy
% measurements
testProposition4 = true;

% Parameters to change the frequency of measurement updates
numObs = 100;
obsPeriod = 1;

% Number of subgraphs
numSubgraph = 1;

% Define the search space for R and Q scales
variables = [optimizableVariable('omegaRScale', [0.1, 1.9]);
             optimizableVariable('omegaQScale', [0.1, 1.9])];

acquisitionFuncs = {'expected-improvement-per-second-plus', ...
    'expected-improvement', 'expected-improvement-plus', ...
    'expected-improvement-per-second', 'lower-confidence-bound', ...
    'probability-of-improvement'};

acquisitionFunc = acquisitionFuncs{1};

% Perform Bayesian optimisation
results = bayesopt(@(x) targetFunction(x, numberOfTimeSteps, numberOfEpisodes, testProposition4, numObs, obsPeriod, numSubgraph), variables, ...
    'AcquisitionFunctionName', acquisitionFunc);

% Path to store results
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

if testProposition4 == true
    prop = "_prop4";
else
    prop = "";
end

fileName = system_name + "_results_" + num2str(numberOfTimeSteps) + "-" + num2str(numberOfEpisodes) + "_" + acquisitionFunc + prop + ".mat";
if saveResults == true
    save(basePath + fileName, 'results');
end

% Define the target function, which takes omega scales, and returns the calculated C value.
function cVal = targetFunction(x, numberOfTimeSteps, numberOfEpisodes, testProposition4, numObs, obsPeriod, numSubgraph)
    import g2o.core.*;
    import pose_graph_experiments.*;
    import two_d_tracking_model_answer.*;
    
    % Extract the variables
    omegaRScale = x.omegaRScale;
    omegaQScale = x.omegaQScale;

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

    % First run retrieves the graph dimensions
    [chi2SumStore(1), chi2Store(1, :), ~, dimX, dimZ] = ...
        runLinearExample(numberOfTimeSteps, ...
        omegaRScale, omegaQScale, testProposition4, ...
        numObs, obsPeriod, numSubgraph);
    
    parfor r = 2 : numberOfEpisodes
        [chi2SumStore(r), chi2Store(r, :)] = ...
            runLinearExample(numberOfTimeSteps, ...
            omegaRScale, omegaQScale, testProposition4, ...
            numObs, obsPeriod, numSubgraph);
    end


    if (testProposition4 == true)
        N = dimZ - dimX;
    else
        N = dimZ;
    end

    if numSubgraph == 1
        meanChi2 = mean(chi2SumStore);
        covChi2 = cov(chi2SumStore);
        cVal = abs(log(meanChi2/N)) + abs(log(covChi2/(2*N)));
    else
        % Compute the mean and covariance for each subgraph
        C = zeros(1, numSubgraph);
        for i = 1:numSubgraph
            meanChi2 = mean(chi2Store(:, i));
            covChi2 = cov(chi2Store(:, i));
        
            % Compute the Consistency Measurement for each subgraph
            C(i) = abs(log(meanChi2/N)) + abs(log(covChi2/(2*N)));
        end
        cVal = mean(C);
    end
end

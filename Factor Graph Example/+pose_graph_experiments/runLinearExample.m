function [chi2, chi2List, Px, dimX, dimZ] = ...
    runLinearExample(numberOfTimeSteps, omegaRScale, omegaQScale, ...
    testProposition4, numObs, obsPeriods, numSubgraph, R, Q)

    import g2o.core.*;
    import pose_graph_experiments.*;
    import two_d_tracking_model_answer.*;

    % Check if numSubgraph is provided, if not, set it to 1
    if (nargin < 7)
        numSubgraph = 1;
    end

    % Check if obsPeriods is provided, if not, set it to a default value
    if (nargin < 6 || isempty(obsPeriods))
        obsPeriods = ones(1, numSubgraph);
    end

    % Some parameters
    dT = 1;
    sigmaR = 10;
    sigmaQ = 1;

    % Work out the state transition equations for the simulator
    F0=[1 dT; 0 1];
    Q0=[dT^3/3 dT^2/2;dT^2/2 dT] * sigmaQ;

    F = [F0 zeros(2); zeros(2) F0];

    % If Q is not provided, use default
    if nargin < 9
        Q = [Q0 zeros(2); zeros(2) Q0];
    end

    % If R is not provided, use default
    if nargin < 8
        R = eye(2) * sigmaR;
    end

    H = [1 0 0 0;
        0 0 1 0];

    % Work out the information matrices
    omegaR = omegaRScale * inv(R);
    omegaQ = omegaQScale * inv(Q);

    % Ground truth array
    trueX = zeros(4, numberOfTimeSteps);
    z = zeros(2, numberOfTimeSteps);

    % First timestep
    trueX(2, 1) = 0.1;
    trueX(4, 1) = -0.1;
    z(:, 1) = H * trueX(:, 1) + sqrtm(sigmaR) * randn(2, 1);

    % Now predict the subsequent steps
    for k = 2 : numberOfTimeSteps
        trueX(:, k) = F * trueX(:, k - 1) + sqrtm(Q) * randn(4, 1);
        z(:, k) = H * trueX(:, k) + sqrtm(sigmaR) * randn(2, 1);
    end

    timeStepsPerSubgraph = floor(numberOfTimeSteps / numSubgraph);
    remainder = mod(numberOfTimeSteps, numSubgraph);

    chi2 = 0;
    chi2List = [];

    % Initialise dimX and dimZ arrays
    dimX = zeros(1, numSubgraph);
    dimZ = zeros(1, numSubgraph);

    % Initialise covariance cell for each subgraph
    Px = cell(1, numSubgraph);

    for subgraphIndex = 1 : numSubgraph
        % Adjust the number of time steps for the last subgraph
        if subgraphIndex == numSubgraph
            timeStepsPerSubgraph = timeStepsPerSubgraph + remainder;
        end

        % Determine current observation period
        currentObsPeriod = obsPeriods(subgraphIndex);

        % Create the graph
        graph = SparseOptimizer();
        algorithm = GaussNewtonOptimizationAlgorithm();
        graph.setAlgorithm(algorithm);

        % This array contains the set of vertices for the target state over time
        v = cell(timeStepsPerSubgraph, 1);

        % This keeps count of the vertex number
        idx = 1;

        % This is the last timestep when a vertex was created
        lastN = 0;

        % Now create the vertices and edges
        for n = 1 : timeStepsPerSubgraph
            takeObservation = (n==1) || ...
                ((subgraphIndex-1)*timeStepsPerSubgraph + n < numObs || ...
                rem((subgraphIndex-1)*timeStepsPerSubgraph + n, currentObsPeriod) == 0);

            % Skip if no observation is taken
            if (takeObservation == false)
                continue;
            end

            % Create the new vertex object
            v{idx} = ObjectStateVertex();

            % Set the initial estimate.
            if (testProposition4 == false)
                v{idx}.setEstimate(trueX(:, (subgraphIndex-1)*timeStepsPerSubgraph + n));
            else
                v{idx}.setEstimate(0*trueX(:, (subgraphIndex-1)*timeStepsPerSubgraph + n));
            end
            
            % Added the vertex to the graph.
            graph.addVertex(v{idx});
            
            % If this isn't the first vertex, add the dynamics. The
            % prediction interval is a function of the time since the last
            % vertex was created
            if (idx > 1)
                processModelEdge = ObjectProcessModelEdge();
                processModelEdge.setVertex(1, v{idx-1});
                processModelEdge.setVertex(2, v{idx});
                processModelEdge.setMeasurement([0;0;0;0]);
                
                dTp = dT * (n - lastN);

                F1=[1 dTp; 0 1];
                Q1=[dTp^3/3 dTp^2/2;dTp^2/2 dTp] * sigmaQ;

                processModelEdge.setF(blkdiag(F1, F1));
                processModelEdge.setInformation(omegaQScale * inv(blkdiag(Q1, Q1)));
                graph.addEdge(processModelEdge);

            end
            
            % Create the measurement edge
            e = ObjectMeasurementEdge();
            
            % Link it so that it connects to the vertex we want to estimate
            e.setVertex(1, v{idx});
            
            % Set the measurement value and the measurement covariance
            e.setMeasurement(z(:,(subgraphIndex-1)*timeStepsPerSubgraph + n));
            e.setInformation(omegaR);
            
            % Add the edge to the graph; the graph now knows we have these edges
            % which need to be added
            graph.addEdge(e);

            % Bump the current vertex index
            idx = idx + 1;

            % Store the time step the observation was taken
            lastN = n;

        end

        % Graph construction complete
        
        % Initialise the optimization. This is done here because it's a bit
        % expensive and if we cache it, we can do it multiple times
        graph.initializeOptimization();
        
        % Optimize
        if (testProposition4 == true)
            graph.optimize();
        end
        
        % Compute the chi2 value
        [chi2_subgraph, ~] = graph.chi2();
        
        chi2 = chi2 + chi2_subgraph;
        chi2List = [chi2List; chi2_subgraph];
        
        if (nargout == 5)
            edges = graph.edges();
            vertices = graph.vertices();
    
            % Get the covariance of vertices
            [~, Px{subgraphIndex}] = graph.computeMarginals();
    
            for e = 1 : length(edges)
                dimZ(subgraphIndex) = dimZ(subgraphIndex) + edges{e}.dimension();
            end
    
            for v =  1 : length(vertices)
                dimX(subgraphIndex) = dimX(subgraphIndex) + vertices{v}.dimension();
            end
        end
    end
end


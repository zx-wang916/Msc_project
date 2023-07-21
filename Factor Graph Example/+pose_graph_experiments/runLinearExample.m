function [chi2, chi2List, edges, dimX, dimZ] = ...
    runLinearExample(numberOfTimeSteps, omegaRScale, omegaQScale, ...
    testProposition4, numObs, obsPeriod, numSubgraph)

    import g2o.core.*;
    import pose_graph_experiments.*;
    import two_d_tracking_model_answer.*;
    
    % Check if numSubgraph is provided, if not, set it to 1
    if (nargin < 7)
        numSubgraph = 1;
    end
    
    % Add measurement edge to every vertex by default
    if (nargin < 6)
        numObs = numberOfTimeSteps;
        obsPeriod = 1;
    end
    
    % Some parameters
    dT = 1;
    sigmaR = 10;
    sigmaQ = 1;
    
    % Work out the state transition equations
    F0=[1 dT; 0 1];
    Q0=[dT^3/3 dT^2/2;dT^2/2 dT] * sigmaQ;
    
    F = [F0 zeros(2); zeros(2) F0];
    Q = [Q0 zeros(2); zeros(2) Q0];
    R = eye(2) * sigmaR;
    
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

    for subgraphIndex = 1 : numSubgraph
        % Adjust the number of time steps for the last subgraph
        if subgraphIndex == numSubgraph
            timeStepsPerSubgraph = timeStepsPerSubgraph + remainder;
        end
        
        % Create the graph
        graph = SparseOptimizer();
        algorithm = GaussNewtonOptimizationAlgorithm();
        graph.setAlgorithm(algorithm);
        
        % This array contains the set of vertices for the target state over time
        v = cell(timeStepsPerSubgraph, 1);
        
        % Now create the vertices and edges
        for n = 1 : timeStepsPerSubgraph
            
            % Create the first object state vertex
            v{n} = ObjectStateVertex();
        
            % Set the initial estimate.
            if (testProposition4 == false)
                v{n}.setEstimate(trueX(:, (subgraphIndex-1)*timeStepsPerSubgraph + n));
            else
                v{n}.setEstimate(0*trueX(:, (subgraphIndex-1)*timeStepsPerSubgraph + n));
            end
            
            % Added the vertex to the graph.
            graph.addVertex(v{n});
            
            % If this isn't the first vertex, add the dynamics
            if (n > 1)
                processModelEdge = ObjectProcessModelEdge();
                processModelEdge.setVertex(1, v{n-1});
                processModelEdge.setVertex(2, v{n});
                processModelEdge.setMeasurement([0;0;0;0]);
                processModelEdge.setF(F);
                processModelEdge.setInformation(omegaQ);
                graph.addEdge(processModelEdge);
            end
            
            if ((subgraphIndex-1)*timeStepsPerSubgraph + n < numObs || rem((subgraphIndex-1)*timeStepsPerSubgraph + n, obsPeriod) == 0)
                % Create the measurement edge
                e = ObjectMeasurementEdge();
                
                % Link it so that it connects to the vertex we want to estimate
                e.setVertex(1, v{n});
                
                % Set the measurement value and the measurement covariance
                e.setMeasurement(z(:,(subgraphIndex-1)*timeStepsPerSubgraph + n));
                e.setInformation(omegaR);
                
                % Add the edge to the graph; the graph now knows we have these edges
                % which need to be added
                graph.addEdge(e);
            end
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
            dimX = 0;
            dimZ = 0;
            edges = graph.edges();
            vertices = graph.vertices();
    
            for e = 1 : length(edges)
                dimZ = dimZ + edges{e}.dimension();
            end
    
            for v =  1 : length(vertices)
                dimX = dimX + vertices{v}.dimension();
            end
        end
    end
end

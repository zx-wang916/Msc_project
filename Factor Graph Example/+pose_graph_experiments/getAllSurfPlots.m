function getAllSurfPlots(os, weekNum, omegaRScaleArray, omegaQScaleArray)

    disp("Plotting ...")
    % Path to read files
    if os == "mac"
        basePath = "~/Desktop/week" + weekNum + "/";
    else
        basePath = "D:\\University\\UCL\\project\\week" + weekNum + "\\";
    end

    % Get all CSV files in the folder
    allFiles = dir(basePath + "*.csv");
    fileNames = {allFiles.name};

    % Initialise processed files list
    processedFiles = {};

    % Loop through all files
    for i = 1:numel(fileNames)
        % Only consider files starting with "C_"
        if startsWith(fileNames{i}, 'C_')
            % Define the file path
            filePath = basePath + fileNames{i};
            % Ignore non-existing files
            if ~isfile(filePath)
                continue;
            end
            
            % Read values
            C = readmatrix(filePath);
            
            % Extract system_name from the file name
            splitName = split(fileNames{i}, '_');
            if numel(splitName) == 2
                % If there is only one underscore, extract the substring between underscore and ".asv"
                system_name = extractBefore(splitName{2}, '.');
            else
                % If there are two or more underscores, extract the string between the first and second underscore
                system_name = splitName{2};
            end
            % Handle the capitalization for system_name
            if system_name == "gps"
                system_name = string(upper(system_name));
            else
                system_name = string(upper(system_name(1))) + string(lower(system_name(2:end)));
            end

            % Check if file name contains 'prop4'
            if contains(fileNames{i}, 'prop4')
                prop = ' (Proposition 4)';
            else
                prop = ' (Proposition 3)';
            end
            
            % Check if file name contains 'measurement_rate'
            if contains(fileNames{i}, 'measurement_rate')
                splitRate = split(splitName{5}, '-');
                numObs = splitRate{1};
                obsPeriod = splitRate{2};
                measure_rate = " Measurement Rate " + num2str(numObs) + "/" + num2str(obsPeriod);
            else
                measure_rate = "";
            end

            % Plotting
            figure(i)
            surf(omegaRScaleArray, omegaQScaleArray, C);
            colorbar; % adds a colorbar, which indicates the scale of C values
            xlabel('omegaRScale');
            ylabel('omegaQScale');
            zlabel('C value');
            title('Consistency Measurement (C) for ' + system_name + measure_rate + prop);
            
            % Compute processed files
            processedFiles = [processedFiles, filePath];
            disp(" => " + length(processedFiles) + " Finished")
        end
    end
    disp("Finish Plotting")
    disp(length(processedFiles) +" Figures generated!")
end

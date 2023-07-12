% Define the range and step for Omega scales
omegaRScaleArray = 0.1:0.1:1.9;
omegaQScaleArray = 0.1:0.1:1.9;

% Read values
C_measure_rate_20_2 = readmatrix('~/Desktop/week9/C_measure_rate_20-2.csv');
C_measure_rate_20_2_prop4 = readmatrix('~/Desktop/week9/C_measure_rate_20-2_prop4.csv');
C_gps = readmatrix('~/Desktop/week9/C_gps.csv');
C_gps_prop4 = readmatrix('~/Desktop/week9/C_gps_prop4.csv');

%% Plots for linear system
figure(1)
surf(omegaRScaleArray, omegaQScaleArray, C_measure_rate_20_2);
colorbar; % adds a colorbar, which indicates the scale of C values
xlabel('omegaRScale');
ylabel('omegaQScale');
zlabel('C value');
title('Consistency Measurement (C) for Linear System Measurement Rate 20/2 (Proposition 3)');

figure(2)
surf(omegaRScaleArray, omegaQScaleArray, C_measure_rate_20_2_prop4);
colorbar; % adds a colorbar, which indicates the scale of C values
xlabel('omegaRScale');
ylabel('omegaQScale');
zlabel('C value');
title('Consistency Measurement (C) for Linear System Measurement Rate 20/2 (Proposition 4)');


%% Plot for GPS system
figure(3)
surf(omegaRScaleArray, omegaQScaleArray, C_gps);
colorbar; % adds a colorbar, which indicates the scale of C values
xlabel('omegaRScale');
ylabel('omegaQScale');
zlabel('C value');
title('Consistency Measurement (C) for GPS System (Proposition 3)');

figure(4)
surf(omegaRScaleArray, omegaQScaleArray, C_gps_prop4);
colorbar; % adds a colorbar, which indicates the scale of C values
xlabel('omegaRScale');
ylabel('omegaQScale');
zlabel('C value');
title('Consistency Measurement (C) for GPS System (Proposition 4)');

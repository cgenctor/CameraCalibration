%% CRV_17
% name: Candas Genctor

%% clean up
clear 
close all;
clc;
%% Load camera parameters
load('WScameraparameters.mat')
load('fisheyeIntr.mat')
%% Read the image
filename = uigetfile('*.*', 'Select the image'); 
rgb = imread(filename);
% figure; imshow(rgb);
% title('Input Image');
%% Rectify image
[im, ~] = undistortImage(rgb, cameraParams, 'OutputView', 'full');
% figure; imshow(im);
% title('Undistorted/Rectified Image');
%% Use circle hough transform to detect the coins in the rectified image.
I = rgb2gray(im);       %rgb to gray scale conversion
%figure; imshow(I);
sigma = 4;  
Iblur = imgaussfilt(I,sigma);    % smoothing
rmin1=70; 
rmax1=86;  
rmin2=105;
rmax2=125;  
rmin3=95;  
rmax3=109;  
[centersM, radiiM] = imfindcircles(Iblur,[rmin1 rmax1],'ObjectPolarity','dark');
[centersL, radiiL] = imfindcircles(Iblur,[rmin2 rmax2],'ObjectPolarity','dark');
[centers10, radii10] = imfindcircles(Iblur,[rmin3 rmax3],'ObjectPolarity','dark');
%% For each circle determine the image coordinates of two points, whose distance is the diameter.
% get number of circles 
centers = [centersM;centers10;centersL];
radii = [radiiM;radii10;radiiL];
numOfCircles = size(centers,1);
% determine 2 coordinates for each circle
x1 = zeros(numOfCircles,1);
y1 = zeros(numOfCircles,1);
x2 = zeros(numOfCircles,1);
y2 = zeros(numOfCircles,1);
for n = 1:numOfCircles
    x1(n) = centers(n,1);
    y1(n) = centers(n,2) + radii(n);
    x2(n) = centers(n,1);
    y2(n) = centers(n,2) - radii(n);
end
topCoordinates = [x1,y1];
bottomCoordinates = [x2,y2];
% plot(x1,y1,'.y')
% plot(x2,y2,'.y')
%% Convert the coordinates of the points to world units and calculate the distance.
% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(I);
% plot(imagePoints(1,1),imagePoints(1,2),'*y')

%%
% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);
%%
% fisheyeIntrinsics used to due to problem with cameraInstrinsics in
% pointToWorld function
coinPoints1 = pointsToWorld(fisheyeIntrinsics_value,R,t,topCoordinates);
coinPoints2 = pointsToWorld(fisheyeIntrinsics_value,R,t,bottomCoordinates);
%%
distances = zeros(numOfCircles,1);
for n = 1:numOfCircles
    distances(n,1) = sqrt((coinPoints1(n,1)-coinPoints2(n,1))^2 + ...
        (coinPoints1(n,2)-coinPoints2(n,2))^2);
end

%% Classify the detected circles based on the diameter & Count how many coins are in each category
% 1 Eurocent 14.5 mm - 17.5 mm
% 10 Eurocent 17.5 mm - 21.5 mm
% 1 Euro-coin 21.5 mm - 25.25 mm
lookup={'1 Eurocent','10 Eurocent','1 Eurocoin'};
numOf1Cent = 0;
numOf10Cent = 0;
numOf1Euro = 0;
for n = 1:numOfCircles 
    Status=lookup{find(histcounts(distances(n,1),[14.5,17.5,21.5,25.25]))};
    %disp("Coin: " + Status);
    if isequal(Status ,'1 Eurocent')
        numOf1Cent = numOf1Cent + 1;
    elseif isequal(Status ,'10 Eurocent')
        numOf10Cent = numOf10Cent + 1;
    elseif isequal(Status ,'1 Eurocoin')
        numOf1Euro = numOf1Euro + 1;
    else 
        msg = 'Error occurred.';
        error(msg)
    end
end
% give the result in the title of the figure.
%% Create a figure, show the image and visualize the detected coins
figure()
imshow(I)
str1 = sprintf('%d ', numOf1Cent);
str2 = sprintf('%d ', numOf10Cent);
str3 = sprintf('%d ', numOf1Euro);
title([num2str(str1),'x 1 Eurocent(s), ',num2str(str2),'x 10 Eurocent(s), ' ...
    ,num2str(str3),'x 1 Eurocoin(s)'])
hold on;
viscircles(centersM, radiiM,'EdgeColor','b');
viscircles(centersL, radiiL,'EdgeColor','g');
viscircles(centers10, radii10,'EdgeColor','r');
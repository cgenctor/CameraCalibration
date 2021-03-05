%% CRV_16_Ruler
% name: Candas Genctor

%% clean up
clear 
close all;
clc;
%% Load camera parameters
load('WScameraparameters.mat')
%% Read the image

filename = uigetfile('*.*', 'Select the image'); 
rgb = imread(filename);
% figure; imshow(rgb);
% title('Input Image');

%% Rectify image

[im, ~] = undistortImage(rgb, cameraParams, 'OutputView', 'full');
% figure; imshow(im);
% title('Undistorted Image');

%% Extract extrinsic for specific position

% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);
% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);

%% Fancy Ruler

% Show the image
figure; imshow(im);

% Set initial coordinates for the measurement points.
h = gca();
imagePoints = [round(0.4*h.XLim(2)), round(0.5*h.YLim(2)); round(0.6*h.XLim(2)), round(0.5*h.YLim(2))];

% Initialize title
updateTitle(cameraParams, R, t, imagePoints)

% Create draggable points
h1 = impoint(gca,imagePoints(1,1),imagePoints(1,2));
h2 = impoint(gca,imagePoints(2,1),imagePoints(2,2));

% Define functions to call when points have been moved
addNewPositionCallback(h1,@(h) updateP(h,h1,h2,cameraParams, R, t));
addNewPositionCallback(h2,@(h) updateP(h,h1,h2,cameraParams, R, t));

%% Helper functions

function [] = updateP(~,h1,h2,cameraParams, R, t)
imagePoints = [getPosition(h1); getPosition(h2)];
updateTitle(cameraParams, R, t, imagePoints);
end

function [] = updateTitle(cameraParams, R, t, imagePoints)
worldPoints = pointsToWorld(cameraParams, R, t, imagePoints);
d = sqrt(sum((worldPoints(1,:)-worldPoints(2,:)).^2));
title(sprintf('distance: %1.0f mm',d),'FontSize',14);
end
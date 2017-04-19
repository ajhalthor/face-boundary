%%
% This file find the tip of nose, plots the boundary of face and plots points along
% one half of the face at intervals of 2 degrees. In the following code :
%
%  - Tip of nose: [nx, ny]
%  - Boundary of face: B
%  - Points on boundary as Vector: [px, py] 
%
%
% Note the boundaries are well defined when the exposure across the face is uniform 
% i.e. there are no shadows on the face. However, even if one half of the face has 
% been plotted abruptly, we can use the other half to replicate the boundary. 
%%

clear all;
close all;

I = imread('images/photo_3.jpg');
I = imresize(I,240/size(I, 1)); % resize all the images to the same size

nose_detector = vision.CascadeObjectDetector('Nose');
mouth_detector = vision.CascadeObjectDetector('Mouth');
eye_detector = vision.CascadeObjectDetector('EyePairSmall');   

%Adjusted according to image size
nose_detector.MergeThreshold = 20;
mouth_detector.MergeThreshold = 15;

% Draw box around the nose & mouth
nbox = step(nose_detector, I); 
mbox = step(mouth_detector, I);

% Guess the first box is correct
nbox = nbox(1,:); 
mbox = mbox(1,:);

% extend the box to include the mouth (Other method)
% nbox(1) = nbox(1) - 0.1*nbox(3);
% nbox(3) = 1.2*nbox(3);
% nbox(4) = 1.5*nbox(4);

% extend the box to include the mouth
final_box = ones(size(nbox));
final_box(1) = min(nbox(1),mbox(1));
final_box(2) = min(nbox(2),mbox(2));
final_box(3) = max(nbox(3),mbox(3));
final_box(4) = nbox(4) + mbox(4);
% final_box(4) = mbox(2) + mbox(4) - nbox(2);

ebox = step(eye_detector, I); % box around the eyes
% extend the eye box to include the eyebrows
ebox(2) = ebox(2) - 0.5*ebox(4);
ebox(4) = 1.5*ebox(4);

%  Find center of nose Haar box
nx = nbox(1) + nbox(3)/2;
ny = nbox(2) + nbox(4)/2;

%  Plot the original image
figure
subplot(2,3,1);
imshow(I);
hold on;
title('Original Image');

% Indicate the nose, mouth & eye regions
rectangle('Position',nbox,'EdgeColor', 'b')
rectangle('Position',ebox,'EdgeColor', 'r')
rectangle('Position',final_box,'EdgeColor', 'g')

%replace nbox with final_box from here on...
nbox = final_box;

% Create a filter for the detected parts of the face (eye, mouth and nose)
maskFilter = uint8(ones(size(I(:,:,1))));
maskFilter(nbox(2):(nbox(2)+nbox(4)), nbox(1):(nbox(1)+nbox(3))) = 0;
maskFilter(ebox(2):(ebox(2)+ebox(4)), ebox(1):(ebox(1)+ebox(3))) = 0;

% convert to grayscale
I_gray = rgb2gray(I);
% Filter high frequency noise
I_gray = imfilter(I_gray, fspecial('gaussian', [3,3], 0.5));

% Plot the filtered grayscale image
subplot(2,3,2); imshow(I_gray);
title('Gray');

% calculate second order derivatives (laplacian of gaussians)
f = fspecial('log',[5 5], 0.3);
I_filtered = imfilter(I_gray, f);
I_filtered = I_filtered.*maskFilter; % exclude the detected parts of the face

% Plot the laplacian of gaussians
subplot(2,3,3); imshow(I_filtered);
title('LoG');

% Apply thresshold to LoG
I_bin = I_filtered < 40;
seDiskNoise = strel('disk',1);
seDiskClose = strel('disk',10);
I_bin1 = imerode(imdilate(I_bin,seDiskNoise), seDiskNoise); % remove noise from the face
I_bin2 = imerode(I_bin1,seDiskClose); % close the boundaries of the face
I_bin3 = imdilate(I_bin2,seDiskClose); % reverse the erode (not used for processing)

subplot(2,3,4); imshow(I_bin1); title('LoG > 50');
subplot(2,3,5); imshow(I_bin2); title('eroded');
subplot(2,3,6); imshow(I_bin3); title('dilated');

CC = bwconncomp(I_bin2); % calculate the regions in the binary image

% search the region containing the nose
ni = sub2ind(size(I_gray), round(ny), round(nx));
for i=1:length(CC.PixelIdxList)
  if any(CC.PixelIdxList{i}==ni)
    iPhase = i;
  end
end

% create a mask for the full face
maskFace = zeros(size(I_gray));
maskFace(CC.PixelIdxList{iPhase}) = 1;
% undo the erosion
maskFace = imdilate(maskFace, seDiskClose);

% visualise the face region
subplot(2,3,1);
visboundaries(maskFace,'Color','b');

% remove all the extrusions and inner regions of the face region
seDisk2 = strel('disk',40);
maskFace = imerode(imdilate(maskFace, seDisk2), seDisk2);

% draw the final face region in red
subplot(2,3,1);
visboundaries(maskFace,'Color','r');
title('Final is in red');


%% Get Coordinates of face boundary

figure;
[B, L] = bwboundaries(maskFace, 'noholes');
imshow(label2rgb(L, @jet, [.5 .5 .5]));
hold on;
for k = 1:length(B)
    boundary = B{k};
    plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2);
end;


%% Plot face boundary and nose tip

% Plot boundary on original image.
% Plot Line x = nx through nose tip

% Vector of X & Y coordinates
x_curve = B{1}(:,2);
y_curve = B{1}(:,1);

figure;
imshow(I), title('White face boundary');
hold on;
plot(x_curve, y_curve, 'w', 'LineWidth', 2);
plot(nx, ny,'Marker','+','Color','blue', 'LineWidth', 2);


%% From the bottom point to the top point, plot 90 points with a 2 degree spacing along B{1}

step = 2; step_rad = deg2rad(step);

for theta = -90:step:90
    
    theta_rad = deg2rad(theta);
    
    m2 = ( tan(theta_rad) + tan(step_rad) ) / (1 - tan(theta_rad)*tan(step_rad) );
    c2 = ny - m2 * nx;
    
    %find points on line in the box for now
    xr = nx: size(I,1); 
    yr = m2 * xr + c2;
    
    % Find point of intersection between line and curve
    [px, py] = intersections(xr,yr,x_curve,y_curve);
    
    % plot point on face boundary
    plot(px, py, 'ro','Color','red'); 
    
    % In case of multiple intersections
    px = px(1,:); py = py(1,:);
    
    % plot line between those nose tip and face boundary
    line([nx, px],[ny, py],'Color','green');

end;

impixelinfo;


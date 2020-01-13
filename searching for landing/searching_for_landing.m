%%
% sample landing locations of the right-side coastline
sample_points_right =  [266, 1498; 301, 1494; 325, 1462;
    334 ,1444; 353 ,1437; 356, 1417;
    375 ,1410; 384 ,1391; 382, 1391;
    394 ,1375; 401 ,1356; 401, 1336;
    423 ,1339; 438 ,1325
]; 

sample_points_left = [
    103, 331; 67 , 257;
    125, 469; 141, 481;
    125, 517; 107, 545;
    267, 110; 341, 117;
    428, 92 ; 451, 120
    ];

sample_points_up = [
    151, 1200; 129 , 945;
    124, 1064; 111, 1144;  
    130, 864; 124, 1064
    ];

% the location of hospitals
hospitals = [
    218, 1480;   % right-side coastline 
    325, 1100;   % right-side coastline 
    116, 1103;   % up-side coastline 
    157, 1024;   % up-side coastline 
    84,  506];   % left-side coastline 

% the model of aircraft. (here we only select type 'B' and 'F')
aircraft_model = [ 'B', 'F'];

% mission radius of the drones
max_flight_distance_B = 233;
max_flight_distance_F = 140;

img = imread('dst.jpg');
%% choose the landing on the up
% for i = 1:length(sample_points_up)
%     landing_zone = [sample_points_up(i, 1) , sample_points_up(i, 2)]; 
    
%     % check if the hospital is obtained in the circle
%     if distanceCost(landing_zone, [hospitals(3,1), hospitals(3,2)]) <= 233 && distanceCost(landing_zone, [hospitals(4,1), hospitals(4,2)]) <= 233
%         % get the area of the roads
%         area = getRoadArea(img, landing_zone);
%         disp("figure " + i + " : " + area);       
%     end    
% end

%% mark the roads in the up circle
roi_up = getRoiMat(img, [sample_points_up(1, 1) , sample_points_up(1, 2)], 0, 100, 200);
% img_highlight_up = imdivide(img, roi_up);
% imshow(img_highlight_up);

%% delete the roads data which in the circle 
img_without_up = imdivide(img, roi_up);

%% choose the landing on the left
% for i = 1:length(sample_points_left)
%     landing_zone = [sample_points_left(i, 1) , sample_points_left(i, 2)]; 

%     if distanceCost(landing_zone, [hospitals(5,1), hospitals(5,2)]) <= 233
%         area = getRoadArea(img_without_up, landing_zone);
%         disp("figure " + sample_points_left(i, 1) + " : " + area);
%     else
%         disp("figure " + distanceCost(landing_zone, [hospitals(5,1), hospitals(5,2)]));
%     end
% end

%% mark the roads in the left circle
roi_left = getRoiMat(img, [sample_points_left(4, 1) , sample_points_left(4, 2)], 200,0, 100);
img_highlight_up_and_left = imadd(roi_up, roi_left);


% %% for the hospitals on the right
% for i = 1:length(sample_points_right)
%     landing_zone = [sample_points_right(i, 1) , sample_points_right(i, 2)]; 

%     if distanceCost(landing_zone, [hospitals(2,1), hospitals(2,2)]) <= 233 && distanceCost(landing_zone, [hospitals(1,1), hospitals(1,2)]) <= 233
%         area = getRoadArea(img_without_up, landing_zone);
%         disp("figure " + sample_points_right(i, 1) + " : " + area);
%     else
%         disp("figure " + distanceCost(landing_zone, [hospitals(5,1), hospitals(5,2)]));
%     end
% end

%% mark the roads in the right circle
roi_right = getRoiMat(img, [sample_points_right(14, 1) , sample_points_right(14, 2)], 20,200, 20);
img_highlight = imadd(img_highlight_up_and_left, roi_right);
K = imlincomb(0.3, img, 0.7, img_highlight);
figure,imshow(K);

h_up = images.roi.Circle(gca,'Center',[1200, 151],'Radius',233);
h_left = images.roi.Circle(gca,'Center',[481, 141],'Radius',233);
h_right = images.roi.Circle(gca,'Center',[1325, 438],'Radius',233);

%% extract the ROI from the original image
function area = getRoadArea(img, landing_zone)
    grayimg = rgb2gray(img);
    [imgW,imgH] = size(grayimg);
    t = linspace(0, 2*pi, 50);   % approximate circle with 50 points
    radius = 233;                          % radius
    center = [landing_zone(1,2) landing_zone(1,1)]; % circle center

    % get circular mask
    BW = poly2mask(radius*cos(t)+center(1), radius*sin(t)+center(2), imgW, imgH);
    grayROI = immultiply(grayimg,BW);

    % extract the ROI image (save the calculate resources)
    rect = [landing_zone(1, 2) - 233 0  landing_zone(1, 2) + 233 616];
    roi = imcrop(grayROI, rect);
    % calculate the num of the "white" pixels in the ROI (pixel by pixel)
    area = 0;
    [roi_w, roi_h] = size(roi);
    for i = 1:roi_w
        for j = 1:roi_h
            if roi(i, j) > 100
                area = area + 1;
            end
        end
    end
end

%% extract the ROI from the original image
function roi = getRoiMat(img, landing_zone, r, g, b)
    grayimg = rgb2gray(img);
    [imgW,imgH] = size(grayimg);
    t = linspace(0, 2*pi, 50);   % approximate circle with 50 points
    radius = 233;                          % radius
    center = [landing_zone(1,2) landing_zone(1,1)]; % circle center

    % get circular mask
    BW = poly2mask(radius*cos(t)+center(1), radius*sin(t)+center(2), imgW, imgH);
    rgbmask(:,:,1) = BW;
    rgbmask(:,:,2) = BW;
    rgbmask(:,:,3) = BW;
    rgbROI = immultiply(img,rgbmask);
    % calculate the num of the "white" pixels in the ROI (pixel by pixel)
    [roi_w, roi_h, ch] = size(rgbROI);
    for i = 1:roi_w
        for j = 1:roi_h
            if rgbROI(i, j) > 100
                rgbROI(i, j, :) = [; r g b];
            end
        end
    end
    roi = rgbROI;
end

%% distanceCost function
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
end

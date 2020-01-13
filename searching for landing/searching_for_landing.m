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
max_flight_distance_B = 465.69;
max_flight_distance_F = 279.39;

img = imread('dst.jpg');

%% choose the landing on the up
area_max = 0
for i = 1:length(sample_points_up)
    landing_zone = [sample_points_up(i, 1) , sample_points_up(i, 2)]; 
    
    % check if the hospital is obtained in the circle
    if distanceCost(landing_zone, [hospitals(3,1), hospitals(3,2)]) <= 465.69 && distanceCost(landing_zone, [hospitals(4,1), hospitals(4,2)]) <= 465.69
        % get the area of the roads
        area = getRoadArea(img, landing_zone);
        disp("figure " + i + " : " + area);       
    end    
end

%% mark the roads in the circle
roi = getRoiMat(img, [sample_points_up(5, 1) , sample_points_up(5, 2)]);
% K = imlincomb(0.4, img, 0.6, roi);
img_highlight_up = immultiply(img, roi);
figure, imshow(img_highlight_up);
h = images.roi.Circle(gca,'Center',[864, 130],'Radius',465);

%% delete the roads data which in the circle 

img_without_up = imdivide(img, roi);

%% for the hospitals on the left
for i = 1:length(sample_points)
    landing_zone = [sample_points(i, 1) , sample_points(i, 2)]; 

    if distanceCost(landing_zone, [hospitals(1,1), hospitals(1,2)]) <= 465.69 && distanceCost(landing_zone, [hospitals(2,1), hospitals(2,2)]) <= 465.69
        grayROI = getROI(img);
        figure,imshow(grayROI);
    end
end


% %% for the hospitals on the right
% for i = 1:length(sample_points)
%     landing_zone = [sample_points(i, 1) , sample_points(i, 2)]; 
    
% end




%% extract the ROI from the original image
function area = getRoadArea(img, landing_zone)
    grayimg = rgb2gray(img);
    [imgW,imgH] = size(grayimg);
    t = linspace(0, 2*pi, 50);   % approximate circle with 50 points
    radius = 465.69;                          % radius
    center = [landing_zone(1,2) landing_zone(1,1)]; % circle center

    % get circular mask
    BW = poly2mask(radius*cos(t)+center(1), radius*sin(t)+center(2), imgW, imgH);
    grayROI = immultiply(grayimg,BW);

    % extract the ROI image (save the calculate resources)
    rect = [landing_zone(1, 2) - 465 0  landing_zone(1, 2) + 465 616];
    roi = imcrop(grayROI, rect);

    % calculate the num of the "white" pixels in the ROI (pixel by pixel)
    area = 0;
    [roi_w, roi_h] = size(roi);
    for i = 1:roi_w
        for j = 1:roi_h
            if roi(i, j) > 0
                area = area + 1;
            end
        end
    end
end

%% extract the ROI from the original image
function roi = getRoiMat(img, landing_zone)
    grayimg = rgb2gray(img);
    [imgW,imgH] = size(grayimg);
    t = linspace(0, 2*pi, 50);   % approximate circle with 50 points
    radius = 465.69;                          % radius
    center = [landing_zone(1,2) landing_zone(1,1)]; % circle center

    % get circular mask
    BW = poly2mask(radius*cos(t)+center(1), radius*sin(t)+center(2), imgW, imgH);
    rgbmask(:,:,1) = BW;
    rgbmask(:,:,2) = BW;
    rgbmask(:,:,3) = BW;
    rgbROI = immultiply(img,rgbmask);
    % calculate the num of the "white" pixels in the ROI (pixel by pixel)
    [roi_w, roi_h, a, b, c] = size(rgbROI);
    for i = 1:roi_w
        for j = 1:roi_h
            if rgbROI(i, j) > 0
                rgbROI(i, j, :) = [20,20,200];
            end
        end
    end
    roi = rgbROI;
end

%% distanceCost function
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
end

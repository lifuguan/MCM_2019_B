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

% the location of hospitals
hospitals = [
    218, 1480;   % right-side coastline 
    325, 1100;   % right-side coastline 
    116, 1103;
    157, 1024;
    84,  506];

% the model of aircraft. (here we only select type 'B' and 'F')
aircraft_model = [ 'B', 'F'];

% mission radius of the drones
max_flight_distance_B = 465.69;
max_flight_distance_F = 279.39;

img = imread('dst.jpg');

%% for the hospitals on the up
for i = 1:length(sample_points)
    landing_zone = [sample_points(i, 1) , sample_points(i, 2)]; 

    if distanceCost(landing_zone, [hospitals(1,1), hospitals(1,2)]) <= 465.69 && distanceCost(landing_zone, [hospitals(2,1), hospitals(2,2)]) <= 465.69
        grayROI = getROI(img);
        
    end
end

% %% for the hospitals on the right
% for i = 1:length(sample_points)
%     landing_zone = [sample_points(i, 1) , sample_points(i, 2)]; 
    
% end

% %% for the hospitals on the left
% for i = 1:length(sample_points)
%     landing_zone = [sample_points(i, 1) , sample_points(i, 2)]; 

%     if distanceCost(landing_zone, [hospitals(1,1), hospitals(1,2)]) <= 465.69 && distanceCost(landing_zone, [hospitals(2,1), hospitals(2,2)]) <= 465.69
%         grayROI = getROI(img);
%         figure,imshow(grayROI);
%     end
% end


function grayROI = grayROI(img)
    grayimg = rgb2gray(img);
    [imgW,imgH] = size(grayimg);
    t = linspace(0, 2*pi, 50);   % approximate circle with 50 points
    radius = 465.69;                          % radius
    center = [hospitals(1,2) hospitals(1,1)]; % circle center
    % get circular mask
    BW = poly2mask(radius*cos(t)+center(1), radius*sin(t)+center(2), imgW, imgH);
    grayROI = immultiply(grayimg,BW);
end



%% distanceCost function
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
end






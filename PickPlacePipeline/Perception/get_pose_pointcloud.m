%%%%%%%%%FUNCTION TO ESTIMATE POSE OF AN OBJECT USING INTEL DEPTH CAM %%%
function [rgblabel,transforms] = whereizz()

    [xx, color_frame, depth_frame, intrinsics, depth_scaling] = pointcloud_example(); % get the color,depth frames and cam intrinsices from exmaple function.
    pointCloudd = pcread("cubefacemodel.ply"); % Model of cube face to be used by ICP
    % pointCloudd2 = pcread("cubefacemodel2.ply");
    % pointCloudd3 = pcread("cuboid.ply");
    % pointCloudd4 = pcread("facemodelnonlinear.ply");

    [label, rgblabel, overlayimg, CC, labels] = segment_objects("None", "green", "bgfeb13.png", true, false, color_frame, depth_frame); % perform color segmentation (from last lab's pipeline) and return label for each identified object
    imshow(rgblabel);

    % Display extracted point cloud
    pcshow(pcfromdepth(depth_frame,1/depth_scaling,intrinsics, ColorImage=color_frame, DepthRange=[0 1]), "VerticalAxisDir","Down");
    title("Obtained Point Cloud")

    % to store tranforms of all objects wrt camera frame
    transforms = [];


    % For each identified box do:
    for box = 1:CC.NumObjects
        
     label = labels(:,:,box);
     rgblabel = label2rgb(label, "jet", "k", "noshuffle");
     % figure;
     % imshowpair(label, color_frame, "montage");

     
    
    % generate point cloud and assign segmentation mask for color
    ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics, ColorImage=rgblabel, DepthRange=[0 1]);
    

    % Fit the largest plane to remove it
    maxDistance = 0.01; % Adjust based on noise level
    [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud, maxDistance);


    % Remove inliers (points on the plane) and keep otehr points
    remainingPoints = select(ptCloud, outlierIndices, "OutputSize","full");
    removed = select(ptCloud, inlierIndices, "OutputSize","full");
    % ensure each point cloud is of same size

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % IMprovement
    % COde needed to add non balck plane points to remaining points
    % the points on face of cube which are colored black shoudl be
    % colroed...
    % nan_mask = any(isnan(removed.Location),3);
    % black_mask = all(removed.Color == 0, 3);
    % remove_indices = nan_mask & black_mask;
    % 
    % loc = ptCloud.Location;
    % col = ptCloud.Color;
    % 
    % 
    % % % size(nan_mask)
    % % % size(removed.Location)
    % % % loc(remove_indices, :) = NaN;
    % % % col(remove_indices, :) = 0;
    % 
    % ptCloudnew = pointCloud(loc, "Color",col);
    


    % Create a new point cloud without the plane
    
    % Display point cloud for DEBUGGING
    figure;
    pcshow(ptCloud,'VerticalAxisDir','Down', "BackgroundColor","w");
    title(["Color Segmented Point Cloud for Object", num2str(box)])
    % 
    % figure;
    % pcshow(remainingPoints, 'VerticalAxisDir','Down', "BackgroundColor","w");
    % title("plane removed");
    % 
    % figure;
    % pcshow(removed, 'VerticalAxisDir','Down', "BackgroundColor","w");
    % title("plane")

    loc = remainingPoints.Location;
    col = remainingPoints.Color;

    % make a mask whic removes all points asssigned black color
    nan_mask = all(remainingPoints.Color == 0,3);
    
    mask3D = repmat(nan_mask, [1, 1, 3]);

    % Apply the mask and form a pointcloud which only has colored cuboid/cube.
    loc(mask3D) = NaN;
    col(mask3D) = 0;

    ptCloudnew = pointCloud(loc, "Color",col);

    % figure;
    % pcshow(ptCloudnew, 'VerticalAxisDir','Down');
    % title(["Segmented box:", num2str(box)]);


    % extract the top face plane of the cuboid
    maxDistance = 0.0025;
    [model, inlierIndices, outlierIndices] = pcfitplane(ptCloudnew, maxDistance, [0 0 1]);
    topface = select(ptCloudnew, inlierIndices, "OutputSize","full");

    % flatten the top face (non flattened can also be used with teh
    % parabolic face pointcloud face model)

    loc = topface.Location;
    col = topface.Color;

    mean_z = nanmean(loc(:,:,3),"all");
    mask = ~isnan(loc(:,:,3));
    loc(:,:,3) = mean_z .* mask + loc(:,:,3) .* ~mask;

    topfacenew = pointCloud(loc, "Color",col);

    % figure;
    % pcshow(topfacenew, 'VerticalAxisDir','Down');
    % title(["topface flattened of box:", num2str(box)]);
    % 
    % 
    % figure;
    % pcshow(topface, 'VerticalAxisDir','Down');
    % title(["topface of Box:", num2str(box)]);
    % 
    % 
    % figure;
    % pcshow(pointCloudd, "VerticalAxisDir","Down");
    % title("Mathematical Model");

        
    fixedDownsampled = pcdownsample(pointCloudd,"gridAverage",0.003);
    movingDownsampled = pcdownsample(topfacenew,"gridAverage",0.003);


    % find transform using ICP
    [tform, movingreg] = pcregistericp(fixedDownsampled, movingDownsampled, "MaxIterations",100);


    % transform the model to and plot to check the accracy of
    % transofmration
    tformed = pctransform(pointCloudd, tform);

    figure;
    pcshowpair(tformed, topfacenew);
    transforms = [transforms tform];
    

    end
    % imshow(clf);

end






%%%%%%%%%%%%%%%%%%
function pose_m = pixel_to_meters(pose_px, half_board_dim, board_edge_px)
    del_x = half_board_dim(1) / board_edge_px(1);
    del_y = half_board_dim(2) / board_edge_px(1);

    pose_m = [pose_px(1)*del_x pose_px(2)*del_y];


end


function T = homogenous_transform(pose)
    x = pose(1);
    y = pose(2);
    z = pose(3);
    theta = pose(4);

    T = [cos(theta), -sin(theta) 0 x;
         sin(theta),  cos(theta) 0 y;
         0 0 1 z;
         0 0 0 1];

end



function [label, rgblabel, overlayimg, CC, labels] = segment_objects(img_path, color, bg_path, bgFilter, from_path, cf, df)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% img_path: input file if cam is not live
% from_path: (bool) true if img is to be laoded from live path. False is
% camera is connected
% color: {red,gree,blue,all}: The color that should be detected
% bg_path: (optional): If an image wothout cube is available, it enhance
% segmetation
% bg_Filter (bool): true if bg path is available.
% cam_center: The center of camera in pixels.
    % Reading the image
    if from_path == true
        img1 = imread(img_path);
    else
        img1 = cf;
        dimg = df;
    end
    % imshow(dimg);
    % img1 = img;
    % imshow(img1);
    bg = imread(bg_path);

    % Convert images to double precision for accurate subtraction
    bg = double(bg);
    fg = double(img1);

    % Perform subtraction for each RGB channel
    diffR = abs(fg(:,:,1) - bg(:,:,1));
    diffG = abs(fg(:,:,2) - bg(:,:,2));
    diffB = abs(fg(:,:,3) - bg(:,:,3));

    % Normalize differences
    diffR = diffR / max(diffR(:));
    diffG = diffG / max(diffG(:));
    diffB = diffB / max(diffB(:));

    % Create binary masks for each channel using adaptive thresholds
    maskR = diffR > 0.3;  % Adjust threshold as needed
    maskG = diffG > 0.3;
    maskB = diffB > 0.3;

    % Combine masks (logical OR) to detect foreground
    bw_mask = maskR | maskG | maskB;

    % Clean the mask with morphological operations
    bw_mask = imopen(bw_mask, strel('disk', 3)); % Remove noise
    bw_mask = imfill(bw_mask, 'holes'); % Fill small gaps

    % Apply mask to each RGB channel

    % Convert back to uint8
    fg_no_bg = uint8(fg); 
    for c = 1:3
        fg_no_bg(:,:,c) = uint8(fg(:,:,c) .* bw_mask);
    end
    img2 = img1;
    if bgFilter == true
        img1 = fg_no_bg;
    end
    
    % Converting rgb to LAB
    img1lab = rgb2lab(img1);
    [l a b] = imsplit(img1lab);

    % Converting rgb to HSV
    img1hsv = rgb2hsv(img1);
    [h s v] = imsplit(img1hsv);

    % creating empty masks
    sz = size(img1(:,:,1));
    redmask = zeros(sz);
    yellowmask = zeros(sz);
    greenmask = zeros(sz);
    bluemask = zeros(sz);

    if color == "red" | color == "all"
        redmask = h > 0.9 & s > 0.5 ;
        redmask = imfill(redmask, "holes"); % filling holes
        redmask = bwareaopen(redmask, 200); % size of actual box around 8200
        SE = strel("disk", 3); % creating a disk strel
        redmask = imclose(redmask, SE); % perofrming close operation to smoothen edges
        % imshow(redmask);
    end

    if color == "yellow" | color == "all"
        % h extracts color, s extracts regions of yellow with ihgh saturation and v
        % ignores the dark yellow areas of the camera stand
        yellowmask = s > 0.5 & h < 0.105 & v > 0.45;
        yellowmask = imfill(yellowmask, "holes"); % as above
        yellowmask = bwareaopen(yellowmask, 200);
        SE = strel("disk", 3);
        yellowmask = imclose(yellowmask, SE);
    end

    if color == "green" | color == "all"
        greenmask = a < -10;
        greenmask = imfill(greenmask, "holes"); % as above
        greenmask = bwareaopen(greenmask, 200);
        SE = strel("disk", 3);
        greenmask = imclose(greenmask, SE);
       

    end

    if color == "blue" | color == "all"
        bluemask = b < -28 & v > 0.3 & l < 50;
        bluemask = imfill(bluemask, "holes"); % as above
        bluemask = bwareaopen(bluemask, 200);
        SE = strel("disk", 3);
        bluemask = imclose(bluemask, SE);
    end

    % combining masks

    allcubemask = redmask | greenmask | yellowmask | bluemask;

    % figure;
    % imshow(allcubemask);

    % remove the small non-cube noise masked pixels from amsk
    allcubemask = bwpropfilt(allcubemask, "Area", [100 640*480]);
    % find connected components
    CC = bwconncomp(allcubemask);

    % Read binary mask
    % bw = imbinarize(allcubemask);  % Ensure binary
    
    % Remove noise
    % bw = imclose(allcubemask, strel('rectangle', [5,5])); 
    % bw = imfill(bw, 'holes');
    
    % Find connected components
    % stats = regionprops(bw, 'BoundingBox', 'Orientation', 'ConvexHull', 'PixelList');
    
    % Display image
    % imshow(bw);
    % hold on;
    
    % Store corners
    all_corners = [];
    center_pts = [];
    center_pts_m = [];

    % hold off;

    % finding labels
    label = bwlabel(allcubemask);
    [M,N] = size(label);

    labels = false(M,N, CC.NumObjects );


    for comp = 1:CC.NumObjects
        mask = label == comp;
        labels(:,:,comp) = mask;
        
    % 
    end

    % figure;
    % imshow(allcubemask);

    % converting labels to rgb
    rgblabel = label2rgb(label, "jet","k", "shuffle");

    % Viewing the color segmentated image
    k1 = imoverlay(img2, redmask, "red");
    k2 = imoverlay(k1, greenmask, "green");
    k3 = imoverlay(k2, bluemask, "blue");
    overlayimg = imoverlay(k3, yellowmask, "yellow");

    % overlayimg = labeloverlay(img1, label);
    
end

% modify to make it in meters

function pt = find_pixel_cords(center, image_point)
    x = image_point(1) - center(1);
    y = center(2) - image_point(2);
    pt = [x y];
end

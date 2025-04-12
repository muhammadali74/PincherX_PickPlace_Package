
    center = [1920/2, 1080/2, 0]; % camera and depth intrisics se ayen ge ye

    
    [label, rgblabel, overlayingimg, CC, poses, poses_m] = segment_objects("intro to rob\test_again_Color.png", "all", "intro to rob\bg_7feb_Color_Color.png", true, false, center);
    img = flipud(overlayingimg);
    [img_height, img_width, ~] = size(overlayingimg);

    figure;
    imshow(rgblabel);
    figure;
    imshow(label);
    

    % calculating homogenous transform 
    poses_m

    % cTb = homogenous_transform(poses_m)
    % cTb2 = homogenous_transform(poses_m(2, :))
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Define image boundaries (centered at (0,0))
    x_range = [-img_width/2, img_width/2];
    y_range = [-img_height/2, img_height/2];

    % Create figure and plot the image centered at (0,0)
    figure; hold on;
    axis equal; grid on;
    xlabel('X'); ylabel('Y');

    % Place image with its center at (0,0)
    image('CData', img, 'XData', x_range, 'YData', y_range);
    
    % Adjust axis limits
    xlim(x_range);
    ylim(y_range);

    % Define unit vectors for local coordinate frame
    axis_length = 50; % Adjust as needed
    delta = 1;                       % Distance between consecutive points
    l = 50;                              % Length of square
    N = l/delta;                      % Number of points
    x_local = zeros(4,N);                   % 1st row has x coordinates and 2nd row has y coordinates, 3rd row z coordinates
    x_local(1,:) = 0:delta:l-delta;
    x_local(2,:) = 0;
    x_local(3, :) = 0;                       % Z-coordinate is zero for all
    x_local(4,:) = 1; % Converting coordinates to homogenous coordinates
    y_local = zeros(4,N);                   % 1st row has x coordinates and 2nd row has y coordinates, 3rd row z coordinates
    y_local(1,:) = 0;
    y_local(2,:) = 0:delta:l-delta;
    y_local(3, :) = 0;                       % Z-coordinate is zero for all
    y_local(4,:) = 1; % Converting coordinates to homogenous coordinates

    % Plot each pose
    for i = 1:size(poses, 1)
        x = poses(i, 1);
        y = poses(i, 2);
        z = poses(i, 3);
        theta = poses(i, 4);

        % Rotation matrix for 2D rotation around Z-axis
        T = [cos(theta), -sin(theta) 0 x;
             sin(theta),  cos(theta) 0 y;
             0 0 1 z;
             0 0 0 1];

        % Rotate local axes
        x_rot = (T * x_local);
        y_rot = (T * y_local);

        % Translate to pose (x, y)
        x_world = x_rot;
        y_world = y_rot;

        % Plot the coordinate axes
        plot(x_world(1,:), x_world(2,:), 'k', 'LineWidth', 2); % X-axis (Red)
        plot(y_world(1,:), y_world(2,:), 'w', 'LineWidth', 2); % X-axis (Red)
        % plot([x_world(:,2), y_world(:,2)], 'b', 'LineWidth', 2); % Y-axis (Green)
        plot(x, y, 'bo', 'MarkerSize', 5, 'LineWidth', 2); % Origin (Blue Dot)
    end
    
    hold off;


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



function [label, rgblabel, overlayimg, CC, center_pts, center_pts_m] = segment_objects(img_path, color, bg_path, bgFilter, from_path, cam_center)

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
        [img1, dimg] = get_image();
        img1 = imread(img_path);
    end
    imshow(dimg);
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
    maskR = diffR > 0.4;  % Adjust threshold as needed
    maskG = diffG > 0.4;
    maskB = diffB > 0.4;

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

    % remove the small non-cube noise masked pixels from amsk
    allcubemask = bwpropfilt(allcubemask, "Area", [5000 1080*1920]);
    % find connected components
    CC = bwconncomp(allcubemask);

    % Read binary mask
    % bw = imbinarize(allcubemask);  % Ensure binary
    
    % Remove noise
    bw = imclose(allcubemask, strel('rectangle', [5,5])); 
    bw = imfill(bw, 'holes');
    
    % Find connected components
    stats = regionprops(bw, 'BoundingBox', 'Orientation', 'ConvexHull', 'PixelList');
    
    % Display image
    imshow(bw);
    hold on;
    
    % Store corners
    all_corners = [];
    center_pts = [];
    center_pts_m = [];


for k = 1:length(stats)
    % Get convex hull points (better than bounding box)
    hull_points = stats(k).ConvexHull;
    
    % Perform PCA for orientation correction
    coeff = pca(hull_points);
    rotated_points = hull_points * coeff; % Align with new basis

    % Find min/max points in rotated space
    min_vals = min(rotated_points);
    max_vals = max(rotated_points);
    
    % Define rectangle in the transformed space
    rect_pts = [min_vals(1), min_vals(2);
                max_vals(1), min_vals(2);
                max_vals(1), max_vals(2);
                min_vals(1), max_vals(2)];
    
    % Transform back to original space
    rect_pts = rect_pts / coeff;

     % indetify the higher and lower of teh two points
    if rect_pts(1,2) > rect_pts(2,2)
        edge = [rect_pts(1,:); rect_pts(2,:)];
    else
        edge = [rect_pts(2,:); rect_pts(1,:)];
    end

    % Invert the y cordinates (for beter intuition)
    pt1 = find_pixel_cords([0, 1080], edge(1, :));
    pt2 = find_pixel_cords([0, 1080], edge(2, :));
    pts = [pt1; pt2];

    % dd = norm(pt2 - pt1)
    % angle = acosd(X/dist);

    Y = pts(2,2) - pts(1,2);
    X = pts(2,1) - pts(1,1);
    angle = atan2(Y,X);
    % angle = acosd([X dd]);

    % edge_pts = longer_edge()
    a = mean(rect_pts(:,1));
    b = mean(rect_pts(:,2));

    % Convert center point from pixel to camera cordinates (in pixels)
    pts = find_pixel_cords(cam_center, [a, b]);

    depth = dimg(int32(b),int32(a));

    % comvert the pixel points to meters
    pts_m = pixel_to_meters(pts, [ 0.326 , 0.268 ], [672, 540]);

    pts = [pts depth];
    pts_m = [pts_m depth]

    


    center_pts = [center_pts ; pts, angle];
    center_pts_m = [center_pts_m ; pts_m, angle];

    % finding orientation
    
    % Store corners
    all_corners = [all_corners; rect_pts];
    
    % Draw the rotated rectangle
    plot([rect_pts(:,1); rect_pts(1,1)], [rect_pts(:,2); rect_pts(1,2)], 'r-', 'LineWidth', 2);
    plot(rect_pts(:,1), rect_pts(:,2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    plot(a, b, 'bo', 'MarkerSize', 5, 'LineWidth', 2);
    
    end

    hold off;

    % finding labels
    label = bwlabel(allcubemask);

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

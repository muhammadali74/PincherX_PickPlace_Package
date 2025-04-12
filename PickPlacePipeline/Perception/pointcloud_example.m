function [ptCloud, color_frame, depth_frame, intrinsics, depth_scaling] = pointcloud_example()
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();


    %% Acquire device parameters 
    % Get streaming device's name
    dev = profile.get_device();  

    % Access Depth Sensor
    depth_sensor = dev.first('depth_sensor');
    rgb_sensor = dev.first('roi_sensor');

    % Find the mapping from 1 depth unit to meters, i.e. 1 depth unit =
    % depth_scaling meters.
    depth_scaling = depth_sensor.get_depth_scale();

    depth_sensor.set_option(realsense.option('motion_range'), 20);
    depth_sensor.set_option(realsense.option('confidence_threshold'), 15);

    % Extract the depth stream
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    
    % Get the intrinsics
    depth_intrinsics = depth_stream.get_intrinsics();


    % Set manual parameters for the RGB sensor
    rgb_sensor.set_option(realsense.option('enable_auto_exposure'), 0); % Disable auto exposure
    rgb_sensor.set_option(realsense.option('exposure'), 1000); % Set exposure to 625
    rgb_sensor.set_option(realsense.option('gain'), 16); % Set gain to 16
    rgb_sensor.set_option(realsense.option('backlight_compensation'), 0); % Set backlight compensation to 0
    rgb_sensor.set_option(realsense.option('brightness'), 15); % Set brightness to 15
    rgb_sensor.set_option(realsense.option('contrast'), 60); % Set contrast to 50
    rgb_sensor.set_option(realsense.option('gamma'), 250); % Set gamma to 300
    rgb_sensor.set_option(realsense.option('hue'), 0); % Set hue to 0
    rgb_sensor.set_option(realsense.option('saturation'), 70); % Set saturation to 70
    rgb_sensor.set_option(realsense.option('sharpness'), 50); % Set sharpness to 50
    rgb_sensor.set_option(realsense.option('enable_auto_white_balance'), 0); % Disable auto white balance
    rgb_sensor.set_option(realsense.option('white_balance'), 4600); % Set white balance to 4600

    %% Align the frames and then get the frames
    % Get frames. We discard the first couple to allow
    % the camera time to settle
    for i = 1:5
        fs = pipe.wait_for_frames();
    end
    
    % Alignment is necessary as the depth cameras and RGB cameras are
    % physically separated. So, the same (x,y,z) in real world maps to
    % different (u,v) in the depth image and the color images. To build a
    % point cloud we only need depth image, but if we want the color the
    % cloud then we'll need the other image.

    % Since the two images are of different sizes, we can either align the
    % depth to color image, or the color to depth.
    % Change the argument to realsense.stream.color to align to the color
    % image.
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    
    % Stop streaming
    pipe.stop();

    % Extract the depth frame
    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[ depth.get_width(),depth.get_height()]),[2 1]);
    
    figure;
    imshow(depth_frame);

    % Extract the color frame
    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);

    %% Create a point cloud using MATLAB library
    % Create a MATLAB intrinsics object
    intrinsics = cameraIntrinsics([depth_intrinsics.fx,depth_intrinsics.fy],[depth_intrinsics.ppx,depth_intrinsics.ppy],size(depth_frame));
    
    % % Create a point cloud
    ptCloud = 0;
    % ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics,ColorImage=color_frame);
    % 
    % % Display point cloud
    % pcshow(ptCloud,'VerticalAxisDir','Down');
    
    
end





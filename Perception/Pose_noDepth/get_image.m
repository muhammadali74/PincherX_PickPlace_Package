function [im, ig] = get_image()
    %% Create all objects to be used in this file
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    % Make Colorizer object to prettify depth output
    colorizer = realsense.colorizer();
    % Create a config object to specify configuration of pipeline
    cfg = realsense.config();

    %% Set configuration and start streaming with configuration
    % Stream options are in stream.m; These options tap into the various
    % sensors included in the camera
    streamType = realsense.stream('depth');
    formatType = realsense.format('Distance');
    cfg.enable_stream(streamType, formatType);
    
    % Enable color stream with specified resolution
    streamType = realsense.stream('color');
    formatType = realsense.format('rgb8');
    cfg.enable_stream(streamType, formatType, 640, 480); % Set RGB resolution to 1920x1080

    % Start streaming on an arbitrary camera with chosen settings
    profile = pipe.start();

    %% Acquire and Set device parameters 
    % Get streaming device's name
    dev = profile.get_device();    
    name = dev.get_info(realsense.camera_info.name);

    % Access Depth Sensor
    depth_sensor = dev.first('depth_sensor');

    % Access RGB Sensor
    rgb_sensor = dev.first('roi_sensor');

    % Find the mapping from 1 depth unit to meters
    depth_scaling = depth_sensor.get_depth_scale();

    % Set depth sensor parameters
    depth_sensor.set_option(realsense.option('laser_power'), 16);
    depth_sensor.set_option(realsense.option('accuracy'), 1);
    depth_sensor.set_option(realsense.option('motion_range'), 10);
    depth_sensor.set_option(realsense.option('filter_option'), 5);
    depth_sensor.set_option(realsense.option('confidence_threshold'), 3);

    % Set the control parameters for the depth sensor
    optionType = realsense.option('visual_preset');
    depth_sensor.set_option(optionType, 9); % Midrange preset   

    % Set manual parameters for the RGB sensor
    rgb_sensor.set_option(realsense.option('enable_auto_exposure'), 0); % Disable auto exposure
    rgb_sensor.set_option(realsense.option('exposure'), 625); % Set exposure to 625
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

    %% Align the color frame to the depth frame and then get the frames
    for i = 1:5
        fs = pipe.wait_for_frames();
    end

    % Alignment
    align_to_depth = realsense.align(realsense.stream.color);
    fs = align_to_depth.process(fs);

    % Stop streaming
    pipe.stop();

    %% Depth Post-processing
    depth = fs.get_depth_frame();
    width = depth.get_width();
    height = depth.get_height();

    % Spatial Filtering
    spatial = realsense.spatial_filter(.5, 20, 2, 0);
    depth_p = spatial.process(depth);

    % Temporal Filtering
    temporal = realsense.temporal_filter(.13, 20, 3);
    depth_p = temporal.process(depth_p);

    %% Color Post-processing
    color = fs.get_color_frame();
    color.get_width();
    color.get_height();

    %% Colorize and display depth frame
    depth_color = colorizer.colorize(depth_p);
    data = depth_color.get_data();
    img = permute(reshape(data', [3, depth_color.get_width(), depth_color.get_height()]), [3 2 1]);

    % Display colorized depth image
    imshow(img);
    title(sprintf("Colorized depth frame from %s", name));

    %% Display RGB frame
    data2 = color.get_data();
    im = permute(reshape(data2', [3, color.get_width(), color.get_height()]), [3 2 1]);

    figure;
    imshow(im);
    title(sprintf("Color RGB frame from %s", name));

    %% Depth frame without colorizing    
    data3 = depth_scaling * double(depth_p.get_data());
    ig = permute(reshape(data3', [width, height]), [2 1]);



    % figure;
    % imshow(mat2gray(ig));
end

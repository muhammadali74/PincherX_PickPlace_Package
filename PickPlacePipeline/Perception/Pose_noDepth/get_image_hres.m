function depth_p = img_hres()
    %% Create all objects to be used in this file
    pipe = realsense.pipeline();
    colorizer = realsense.colorizer();
    cfg = realsense.config();

    %% Configure streams
    % Enable depth stream at 640x480
    cfg.enable_stream(realsense.stream('depth'), 640, 480, realsense.format('Z16'));
    % Enable RGB stream at 1920x1080
    cfg.enable_stream(realsense.stream('color'), 1920, 1080, realsense.format('rgb8'));

    % Start pipeline
    profile = pipe.start(cfg);

    %% Acquire and Set device parameters 
    dev = profile.get_device();    
    name = dev.get_info(realsense.camera_info.name);

    % Access depth and RGB sensors
    depth_sensor = dev.first('depth_sensor');
    rgb_sensor = dev.first('roi_sensor');

    % Depth scaling factor
    depth_scaling = depth_sensor.get_depth_scale();

    % Set manual parameters for the RGB sensor
    rgb_sensor.set_option(realsense.option('enable_auto_exposure'), 0);
    rgb_sensor.set_option(realsense.option('exposure'), 625);
    rgb_sensor.set_option(realsense.option('gain'), 16);
    rgb_sensor.set_option(realsense.option('backlight_compensation'), 0);
    rgb_sensor.set_option(realsense.option('brightness'), 15);
    rgb_sensor.set_option(realsense.option('contrast'), 50);
    rgb_sensor.set_option(realsense.option('gamma'), 300);
    rgb_sensor.set_option(realsense.option('hue'), 0);
    rgb_sensor.set_option(realsense.option('saturation'), 70);
    rgb_sensor.set_option(realsense.option('sharpness'), 50);
    rgb_sensor.set_option(realsense.option('enable_auto_white_balance'), 0);
    rgb_sensor.set_option(realsense.option('white_balance'), 4600);

    %% Capture frames
    for i = 1:5
        fs = pipe.wait_for_frames();
    end

    % Align depth to color
    align_to_color = realsense.align(realsense.stream.color);
    fs = align_to_color.process(fs);

    % Stop pipeline
    pipe.stop();

    %% Depth Upsampling
    depth = fs.get_depth_frame();
    
    % Convert depth to disparity for upsampling
    disparity_transform = realsense.disparity_transform(true);
    disparity_frame = disparity_transform.process(depth);

    % Apply spatial filtering to smooth edges
    spatial = realsense.spatial_filter(0.5, 20, 2, 0);
    depth_filtered = spatial.process(disparity_frame);

    % Convert back to depth format
    disparity_transform.set_option(realsense.option('disparity_to_depth'), 1);
    depth_upsampled = disparity_transform.process(depth_filtered);

    % Hole filling filter
    hole_filling = realsense.hole_filling_filter(1);
    depth_p = hole_filling.process(depth_upsampled);

    %% Color Processing
    color = fs.get_color_frame();

    %% Display Upsampled Depth
    depth_color = colorizer.colorize(depth_p);
    depth_data = depth_color.get_data();
    depth_img = permute(reshape(depth_data', [3, 1920, 1080]), [3 2 1]);

    figure;
    imshow(depth_img);
    title(sprintf("Upsampled Depth (1920x1080) from %s", name));

    %% Display RGB Frame
    color_data = color.get_data();
    rgb_img = permute(reshape(color_data', [3, 1920, 1080]), [3 2 1]);

    figure;
    imshow(rgb_img);
    title(sprintf("Color RGB Frame from %s", name));

end

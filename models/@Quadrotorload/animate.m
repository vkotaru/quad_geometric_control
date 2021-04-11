function animate(obj,opts_in)
% function to animate the quadrotor

%%
% default options
opts_default.RATE = 25;
opts_default.data.t = [];
opts_default.data.x = [];
opts_default.data.xd = [];
opts_default.data.td = [];

opts_default.vid.MAKE_MOVIE = 0;
opts_default.vid.filename = 'results/vid1';
opts_default.vid.Quality = 100;
opts_default.vid.FrameRate = 24;

opts_default.frame.move = false;
opts_default.frame.view = {40,25};

xd_flag = false;

%% frame
% initialize the animation figure and axes
    figure_x_limits = [-5 5];
    figure_y_limits = [-5 5];
    figure_z_limits = [-5 5] ;
    fig1 = figure;

    set(0,'Units','pixels')
    scnsize = get(0,'ScreenSize');

    screen_width = scnsize(3);
    screen_height = scnsize(4);

    % find the minimum scaling factor
    figure_x_size = figure_x_limits(2) - figure_x_limits(1);
    figure_y_size = figure_y_limits(2) - figure_y_limits(1);

    xfactor = screen_width/figure_x_size;
    yfactor = screen_height/figure_y_size;

    if (xfactor < yfactor)
      screen_factor = 0.5*xfactor;
    else
      screen_factor = 0.5*yfactor;
    end

    % calculate screen offsets
    screen_x_offset = (screen_width - screen_factor*figure_x_size)/2;
    screen_y_offset = (screen_height - screen_factor*figure_y_size)/2;

    % draw figure and axes
    set(fig1,'Position', [screen_x_offset screen_y_offset...
        1.5*screen_factor*figure_x_size 1.5*screen_factor*figure_y_size]);
    set(fig1,'MenuBar', 'none');
    axes1 = axes;
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
%     set(axes1,'Position',[0 0 1 1]);
%     set(axes1,'Color','w');
%     set(axes1,'TickDir','out');
    axis equal ;
    
  
    %% parse inputs
    opts = struct_overlay(opts_default,opts_in);
    
    if ~isempty(opts.data.xd)
       xd_flag = true; 
    end
    if isempty(opts.data.td)
        if length(opts.data.xd) == length(opts.data.x)
            warning('provide td or length(xd) should be equal to length(x)');
        end
    end
    
    % extract data
    RATE = opts.RATE;
      
    t = opts.data.t;
    x = opts.data.x;
    td = opts.data.td;
    xd = opts.data.xd;
    
    %% data
    
    if isrow(t)
        t = t';
    end
    if size(x,2)>size(x,1)
        x = x';
    end
    if isrow(td)
        td = td';
    end
    if size(xd,2)>size(xd,1)
        xd = xd';
    end
    box on;
    
    [t, x] = even_sample(t, x, RATE);
    t = t+t(1);    
    [~, xd] = even_sample(td,xd,RATE);
    

if(opts.vid.MAKE_MOVIE)
    M = moviein(length(t)) ;

    vidObj = VideoWriter(strcat(opts.vid.filename,'.avi'),'Motion JPEG AVI');
    vidObj.FrameRate = opts.vid.FrameRate;
    vidObj.Quality = opts.vid.Quality;
    open(vidObj);frameRate = vidObj.FrameRate;
    % nFrame = floor(frameRate*t(end));
    frameDelay = 1/frameRate;
    time = 0;
end

hist = 5000 ;

    for i=1:length(t)
        drawQuadrotor(axes1, x(i,:)');
        
        plot3(x(max(1,i-hist):i, 1), x(max(1,i-hist):i, 2), x(max(1,i-hist):i, 3), 'k','linewidth',2) ;
        if xd_flag
            l = plot3(xd(max(1,i-hist):i, 1), xd(max(1,i-hist):i, 2), xd(max(1,i-hist):i, 3), 'r','linewidth',2);
%             l.Color(4) = 0.4;
        end
    %         plot3(x(max(1,i-hist):i, 1)-L*x(max(1,i-hist):i,7), x(max(1,i-hist):i, 2)-L*x(max(1,i-hist):i,8), x(max(1,i-hist):i, 3)-L*x(max(1,i-hist):i,9), 'r') ;
            string = sprintf('Running\n t = %1.2fs/%1.fs \n',t(i), t(end));
            text(x(i,1)-2.5,x(i,2)+2.5,string,'FontAngle','italic','FontWeight','bold');
        drawnow;
        figure_x_limits_ = figure_x_limits+opts.frame.move*x(i,1);
        figure_y_limits_ = figure_y_limits+opts.frame.move*x(i,2);
        figure_z_limits_ = figure_z_limits+opts.frame.move*x(i,3);
        set(axes1,'XLim',figure_x_limits_,'YLim',figure_y_limits_,'ZLim',figure_z_limits_);
        if opts.vid.MAKE_MOVIE
            M(:,i) = getframe; 
            % Write data to video file
            writeVideo(vidObj,getframe(gcf));
            % time step system to next frame:
            time = time + frameDelay;
        end        
        
    end

function drawQuadrotor(parent,x)
    tem = get(parent,'Children');
    delete(tem);
    
    [xL, ~, R, ~,q,~]=obj.unzipState(x);
    xQ = xL-obj.l*q;
    
    s.L = 0.175; %length of quadrotor boom
    s.R = 0.1; %radius of propeller prop

    BRW = R' ;
    point1 = BRW'*[s.L,0,0]';
    point2 = BRW'*[0,s.L,0]';
    point3 = BRW'*[-s.L,0,0]';
    point4 = BRW'*[0,-s.L,0]';

    nprop = 40;
    propangs = linspace(0,2*pi,nprop);
    proppts = s.R*BRW'*[cos(propangs);sin(propangs);zeros(1,nprop)];

    wp = xQ ;
    wp1 = wp + point1;
    wp2 = wp + point2;
    wp3 = wp + point3;
    wp4 = wp + point4;
    wp_cable_attach = wp + BRW'*[0;0;0] ; %[0;0;-params.cable_attach_d];

    prop1 = proppts + wp1*ones(1,nprop);
    prop2 = proppts + wp2*ones(1,nprop);
    prop3 = proppts + wp3*ones(1,nprop);
    prop4 = proppts + wp4*ones(1,nprop);

    lwp = 2 ;
    lw1 = 1 ;
    lwc = 2 ;
    lwl = 50 ;

    s.qhandle1 = line([wp1(1),wp3(1)],[wp1(2),wp3(2)],[wp1(3),wp3(3)]); hold on ;
    s.qhandle2 = line([wp2(1),wp4(1)],[wp2(2),wp4(2)],[wp2(3),wp4(3)]);
    set(s.qhandle1,'Color','k', 'LineWidth',lw1);
    set(s.qhandle2,'Color','k', 'LineWidth',lw1);

    s.hprop1 = plot3(prop1(1,:),prop1(2,:),prop1(3,:),'r-', 'LineWidth',lwp);
    s.hprop2 = plot3(prop2(1,:),prop2(2,:),prop2(3,:),'b-', 'LineWidth',lwp);
    s.hprop3 = plot3(prop3(1,:),prop3(2,:),prop3(3,:),'b-', 'LineWidth',lwp);
    s.hprop4 = plot3(prop4(1,:),prop4(2,:),prop4(3,:),'b-', 'LineWidth',lwp);

    s.hload = scatter3(xL(1,1), xL(2,1), xL(3,1), lwl, 'k', 'filled') ;
    s.cable = plot3([wp_cable_attach(1) xL(1,1)], [wp_cable_attach(2) xL(2,1)], [wp_cable_attach(3) xL(3,1)], 'k') ;
    s.cable_attach = plot3([wp(1) wp_cable_attach(1)], [wp(2) wp_cable_attach(2)], [wp(3) wp_cable_attach(3)], 'r') ;
    grid on

    view(opts.frame.view{:});
    % view([-1,-1,1])
    % view(40,5)
    % view(0,0);
    % view(0,90);

    xlbl = xlabel('$$\textbf{X}$$','Interpreter','Latex'); xlbl.FontSize = 15;
    ylbl = ylabel('$$\textbf{Y}$$','Interpreter','Latex'); ylbl.FontSize = 15;
    zlbl = zlabel('$$\textbf{Z}$$','Interpreter','Latex'); zlbl.FontSize = 15;

end

   
end

%%
function [Et, Ex] = even_sample(t, x, Fs, type)
if nargin < 4, type = 'linear'; end

dt = diff(t);
dt = dt + (dt==0)*1e-5;
t = [0;cumsum(dt)];

% Obtain the process related parameters
N = size(x, 2);    % number of signals to be interpolated
M = size(t, 1);    % Number of samples provided
t0 = t(1,1);       % Initial time
tf = t(M,1);       % Final time
EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with
                   % the specified sampling frequency
Et = linspace(t0, tf, round(EM))';
% Using linear interpolation (used to be cubic spline interpolation)
% and re-sample each signal to obtain the evenly sampled forms
    for s = 1:N
      Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1),type); 
    end
end   



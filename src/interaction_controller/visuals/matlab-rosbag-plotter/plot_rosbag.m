%%
%   rosbag_plotter_indivitual_plots
%
%   This script is specifically written to plot the rosbags that document
%   the *Interaction Controller Physical Tests* , and it is supposed to work
%   on any rosbag from 13-04-2021 and after.
%
%   The output of this script is 11 figures such as:
%   1- Position tracking
%   2- Position error
%   3- Attitude errors
%   4- Attitude errors boxplot
%   5- position errors boxlplot
%   6- Normalized Torque
%   7- Normalized Thrust
%   8- Controller force commands
%   9- Controller torque commands
%   10- Controller force commands boxplot
%   11- Controller torque commands boxplot
%
%   Ayham Alharbat - initiated 17-06-2021
%
%% Check if the required scripts are in the directory
if ~ isfile('parse_rosbag.m')
     error('The script (parse_rosbag.m) is not available in the directory!');
end
%% Add the path of logs
addpath("./../../logs/")
%% Clear & close
close all
clear; clc;
%% Load rosbag & clean-up
bag = rosbag("2023-05-25-11-15-20.bag");
%% Plotting options
settings.plot_start = 0;
settings.plot_end = 440;
% use this to choose the desired starting time for the plot
% or the desired zero for the plots
settings.time_offset = 0;
settings.with_arm = 0;
display.opt.position = [200 200 800 300];
display.opt.fontsize = 20.0;
display.opt.linewidth = 1.5;
display.opt.offset = 0;
display.pfig = []; % it will contain the handlers to all figures plotted
Dimension = [800 700];
saving_path = pwd;
%% Parse the messages
disp('Parsing...');
parse_rosbag;
disp('Finished Parsing.');
disp('Plotting...');
%% Choose which data to plot
choose.plot.position_tracking = 1;
choose.plot.attitude_tracking = 0;
choose.plot.attitude_tracking_individual = 0;
choose.plot.planar_position = 0;
choose.plot.velocity = 0;
choose.plot.position_errors = 1;
choose.plot.attitude_errors = 0;
choose.plot.attitude_boxplot = 0;
choose.plot.position_boxplot = 0;
choose.plot.normal_torque = 0;
choose.plot.normal_thrust = 0;
choose.plot.force_commands = 0;
choose.plot.torque_commands = 0;
choose.plot.force_commands_boxplot = 0;
choose.plot.torque_commands_boxplot = 0;
choose.plot.general_plots = 0;
choose.plot.controller_plots = 0;
choose.plot.optitrakVsEKF2 = 0;
choose.plot.arm_position_tracking = 0;
choose.plot.arm_torque_tracking = 0;
choose.plot.battery_status = 1;
%% Choose which data to print to pdf files
choose.print.position_tracking = 0;
choose.print.attitude_tracking = 0;
choose.print.planar_position = 0;
choose.print.velocity = 0;
choose.print.position_errors = 0;
choose.print.attitude_errors = 0;
choose.print.attitude_boxplot = 0;
choose.print.position_boxplot = 0;
choose.print.normal_torque = 0;
choose.print.normal_thrust = 0;
choose.print.force_commands = 0;
choose.print.torque_commands = 0;
choose.print.force_commands_boxplot = 0;
choose.print.torque_commands_boxplot = 0;
choose.print.general_plots = 0;
choose.print.controller_plots = 0;
choose.print.optitrakVsEKF2 = 0;
choose.print.arm_position_tracking = 0;
choose.print.arm_torque_tracking = 0;
%% Setup the highlighting settings
highlight.t0 = 28;
highlight.t1 = 54;
highlight.t2 = 50;
% transparency
highlight.tran = 0.00;
%% Position tracking plot
if (choose.plot.position_tracking)
    title_str = 'Position tracking';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    set(groot,'defaultAxesTickLabelInterpreter','latex');
    hold on
    plot(plotdata.references.p.Time, plotdata.references.p.Data(:, 1),'--','Color','#0072BD','LineWidth',display.opt.linewidth);
    plot(plotdata.references.p.Time, plotdata.references.p.Data(:, 2), '--','Color','#D95319', 'LineWidth', display.opt.linewidth);
    plot(plotdata.references.p.Time, plotdata.references.p.Data(:, 3), '--','Color','#EDB120', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.p.Time  , plotdata.estimated_states.p.Data(:, 1),'Color','#0072BD','LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.p.Time  , plotdata.estimated_states.p.Data(:, 2),'Color','#D95319','LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.p.Time  , plotdata.estimated_states.p.Data(:, 3),'Color','#EDB120','LineWidth', display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-2 4])
    lh = legend('$p_{r,x}$', '$p_{r,y}$', '$p_{r,z}$', ...
        '$p_{x}$', '$p_{y}$', '$p_{z}$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.position_tracking)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Attitude tracking plot
if (choose.plot.attitude_tracking)
    title_str = 'Attitude tracking';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.references.euler.Time, plotdata.references.euler.Data(:, 1),'--','Color','#0072BD','LineWidth',display.opt.linewidth);
    plot(plotdata.references.euler.Time, plotdata.references.euler.Data(:, 2), '--','Color','#D95319', 'LineWidth', display.opt.linewidth);
    plot(plotdata.references.euler.Time, plotdata.references.euler.Data(:, 3), '--','Color','#EDB120', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 1),'Color','#0072BD','LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 2),'Color','#D95319','LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 3),'Color','#EDB120','LineWidth', display.opt.linewidth);
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-10,10])
    lh = legend('$\phi^r$', '$\theta^r$', '$\psi^r$', ...
        '$\phi$', '$\theta$', '$\psi$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[deg]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.attitude_tracking)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end

%% Attitude tracking plot
if (choose.plot.attitude_tracking_individual)
    title_str = 'Attitude tracking';
    fh = figure('position', [200 200 800 800], 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    sub1 = subplot(3,1,1);
    hold on
    plot(plotdata.references.euler.Time, plotdata.references.euler.Data(:, 1),'--','LineWidth',display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 1),'LineWidth', display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    ylim([-6 3])
    lh = legend('$\phi_r$', ...
        '$\phi$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
%     xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[deg]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sub2 = subplot(3,1,2);
    hold on
    plot(plotdata.references.euler.Time, plotdata.references.euler.Data(:, 2), '--', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 2),'LineWidth', display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-4 highlight.ylimits(2)+4 highlight.ylimits(2)+4 highlight.ylimits(1)-4],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    ylim([-10 8])
    lh = legend('$\theta_r$', ...
        '$\theta$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
%     xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[deg]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sub3 = subplot(3,1,3);
    hold on
    plot(plotdata.references.euler.Time, plotdata.references.euler.Data(:, 3), '--', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 3),'LineWidth', display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    ylim([-30 30])
    lh = legend('$\psi_r$', ...
        '$\psi$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[deg]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    sgtitle(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize)
    display.pfig = [display.pfig fh];
    if (choose.print.attitude_tracking)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end

%% Planar position map - SET LINE COLOR
if choose.plot.planar_position
    title_str = 'Planar position map';
    
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on;
    plot(plotdata.references.p.Data(:, 1), plotdata.references.p.Data(:, 2), '--', 'LineWidth', display.opt.linewidth-1);
    plot(plotdata.estimated_states.p.Data(:, 1), plotdata.estimated_states.p.Data(:, 2), ...
        'LineWidth', display.opt.linewidth);
    hold off
    grid
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlabel('x [m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('y [m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    lh = legend('$p_{xy}^r$', '$p_{xy}$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    display.pfig = [display.pfig fh];
end
%% Velocity plot
if (choose.plot.velocity)
    title_str = 'Linear Velocity';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.estimated_states.v.Time, plotdata.estimated_states.v.Data(:, :),'LineWidth',display.opt.linewidth);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-1,1])
    lh = legend('$v_{x}$', '$v_{y}$', '$v_{z}$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[m/s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.velocity)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Position errors plot
if (choose.plot.position_errors)
    title_str = 'Position errors';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.errors.p.Time, plotdata.errors.p.Data(:, :),'LineWidth',display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-20 highlight.ylimits(2)+20 highlight.ylimits(2)+20 highlight.ylimits(1)-20],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-20 highlight.ylimits(2)+20 highlight.ylimits(2)+20 highlight.ylimits(1)-20],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-2,2])
    lh = legend('$e_{x}$', '$e_{y}$', '$e_{z}$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.position_errors)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Attitude errors plot
if (choose.plot.attitude_errors)
    title_str = 'Attitude errors';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.errors.attitude.Time, plotdata.errors.attitude.Data(:, :),'LineWidth',display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-20 highlight.ylimits(2)+20 highlight.ylimits(2)+20 highlight.ylimits(1)-20],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-20 highlight.ylimits(2)+20 highlight.ylimits(2)+20 highlight.ylimits(1)-20],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-0.5,0.2])
    lh = legend('$e_{x}$', '$e_{y}$', '$e_{z}$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel(' ', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.attitude_errors)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% boxplots for attitude erros
if (choose.plot.attitude_boxplot)
    title_str = 'Attitude errors';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    boxplot(plotdata.errors.attitude.Data(:, :),'Labels',{'Roll','Pitch','Yaw'});grid;
    set(gca, 'FontSize', display.opt.fontsize-6)
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    if (choose.print.attitude_boxplot)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% boxplots for position erros
if (choose.plot.position_boxplot)
    title_str = 'Position errors';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    boxplot(plotdata.errors.p.Data(:, :),'Labels',{'x','y','z'});grid;
    set(gca, 'FontSize', display.opt.fontsize-6)
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    if (choose.print.position_boxplot)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Normalized Torque plot
if (choose.plot.normal_torque && ~isempty(structs.actuator_control))
    title_str = 'Normalized Torque Commands';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.xtorque, "LineWidth", 1.5);
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.ytorque, "LineWidth", 1.5);
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.ztorque, "LineWidth", 1.5);
    grid
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-0.4,0.2])
    lh = legend('$\bar \tau_x$','$\bar \tau_y$','$\bar \tau_z$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel(' ', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.normal_torque)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Normalized thrust plot
if (choose.plot.normal_thrust && ~isempty(structs.actuator_control))
    title_str = 'Normalized thrust Command';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.thrust, "LineWidth", 1.5);
    grid
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-0.1,1])
    lh = legend('$\bar T$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel(' ', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.normal_thrust)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Force Commands plot
if (choose.plot.force_commands)
    title_str = 'Controller Force Commands';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.controller.force_d.Time,plotdata.controller.force_d.Data, "LineWidth", 1.5);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-20 highlight.ylimits(2)+20 highlight.ylimits(2)+20 highlight.ylimits(1)-20],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-20 highlight.ylimits(2)+20 highlight.ylimits(2)+20 highlight.ylimits(1)-20],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    grid
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-30,70])
    lh = legend('$F_x$','$F_y$','$F_z$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[N]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.force_commands)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Torque Commands plot
if (choose.plot.torque_commands)
    title_str = 'Controller Torque Commands';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    hold on
    plot(plotdata.controller.torque_d.Time,plotdata.controller.torque_d.Data, "LineWidth", 1.5);
    grid
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-2,2])
    lh = legend('$\tau_x$','$\tau_y$','$\tau_z$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[N.m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    display.pfig = [display.pfig fh];
    if (choose.print.torque_commands)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% boxplots for torque commands
if (choose.plot.torque_commands_boxplot)
    title_str = 'Torque Commands';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    boxplot(plotdata.controller.torque_d.Data(:, :),'Labels',{'Roll','Pitch','Yaw'});grid;
    set(gca, 'FontSize', display.opt.fontsize-6)
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    if (choose.print.torque_commands_boxplot)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% boxplots for Force Commands
if (choose.plot.force_commands_boxplot)
    title_str = 'Force Commands';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    boxplot(plotdata.controller.force_d.Data(:, :),'Labels',{'x','y','z'});grid;
    set(gca, 'FontSize', display.opt.fontsize-6)
    title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
    if (choose.print.force_commands_boxplot)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% plot General plots
if (choose.plot.general_plots)
    % new figure
    title_str = 'General Plots';
    fh = figure('units','normalized','outerposition',[0 0 0.5 1],'name', title_str);
    % set background to white
    set(gcf,'color','w');
    % first subplot
    sub11 = subplot(4,1,1);
    hold on
    plot(plotdata.references.p.Time,plotdata.references.p.Data(:,:),"LineWidth",1.5)
    hold off
    ylabel("Desired position [m]",'fontsize', 12,'Interpreter','latex')
    xlabel("(a)",'fontsize', 14,'Interpreter','latex')
    lh = legend("$x_d$","$y_d$","$z_d$",'Interpreter','latex',...
        "Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    xlim([settings.plot_start,settings.plot_end])
    grid on
    title('')
    
    % second subplot
    sub12 = subplot(4,1,2);
    hold on
    plot(plotdata.estimated_states.p.Time, plotdata.estimated_states.p.Data(:,:),"LineWidth",1.5)
    hold off
    ylabel("Position [m]",'fontsize', 14,'Interpreter','latex')
    xlabel("(b)",'fontsize',14,'Interpreter','latex')
    lh = legend("$x$","$y$","$z$",'Interpreter','latex',"Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    xlim([settings.plot_start,settings.plot_end])
    grid on
    title('')
    
    % third subplot
    sub13 = subplot(4,1,3);
    hold on
    plot(plotdata.errors.p.Time, plotdata.errors.p.Data(:,:),"LineWidth",1.5)
    hold off
    ylabel("Position error [m]",'fontsize', 12,'Interpreter','latex')
    xlabel("(c)",'fontsize',14,'Interpreter','latex')
    lh = legend("$e_x$","$e_y$","$e_z$",'Interpreter','latex',...
        "Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    xlim([settings.plot_start,settings.plot_end])
    ylim([-2,2])
    grid on
    title('')
    
    % fourth subplot
    sub14 = subplot(4,1,4);
    hold on
    plot(plotdata.errors.attitude.Time,plotdata.errors.attitude.Data(:,:),"LineWidth",1.5)
    hold off
    ylabel("Attitude error on SO(3)",'fontsize', 12,'Interpreter','latex')
    lh = legend("$e_{Rx}$","$e_{Ry}$","$e_{Rz}$",'Interpreter','latex',...
        "Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    xlim([settings.plot_start,settings.plot_end])
    grid on
    title('')
    xlabel({'Time [s]'; '(d)'}, 'fontsize', 14, 'Interpreter','latex')
    sgtitle('General Plots', 'Interpreter','latex')
    if (choose.print.general_plots)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% plot data in the second figure
if (choose.plot.controller_plots && ~isempty(structs.actuator_control))
    title_str = 'Controller Plots';
    fh = figure('units','normalized','outerposition',[0 0 0.5 1],'name', title_str);
    set(gcf,'color','w');
    sub21 = subplot(4,1,1);
    hold on
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.xtorque, "LineWidth", 1.5);
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.ytorque, "LineWidth", 1.5);
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.ztorque, "LineWidth", 1.5);
    grid
    hold off
    xlim([settings.plot_start,settings.plot_end])
    xlabel('(a)','Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    ylabel('Normalized torques','Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    lh = legend('$\bar \tau_x$','$\bar \tau_y$','$\bar \tau_z$','Interpreter',...
        'latex',"Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    
    sub22 = subplot(4,1,2);
    hold on
    plot(plotdata.controller.actuator_control.Time, plotdata.controller.actuator_control.thrust, "LineWidth", 1.5);
    grid
    hold off
    xlim([settings.plot_start,settings.plot_end])
    xlabel('(b)','Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    ylabel('Normalized thrust','Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    lh = legend('$\bar{T}$','Interpreter','latex',"Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    
    sub23 = subplot(4,1,3);
    hold on
    plot(plotdata.controller.force_d.Time, plotdata.controller.force_d.Data(:,:),"LineWidth",1.5)
    hold off
    xlabel('(c)','Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    ylabel("Force commands [N]",'Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    lh = legend('$F_x$','$F_y$','$F_z$','Interpreter','latex',"Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    xlim([settings.plot_start,settings.plot_end])
    title('')
    grid on
    
    sub24 = subplot(4,1,4);
    hold on
    plot(plotdata.controller.torque_d.Time, plotdata.controller.torque_d.Data(:,:),"LineWidth",1.5)
    hold off
    xlim([settings.plot_start,settings.plot_end])
    xlabel({'Time [s]'; '(d)'}, 'Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    ylabel('Torque commands [N.m]','Interpreter','latex', 'fontsize', 14, 'Interpreter','latex')
    lh = legend('$\tau_x$','$\tau_y$','$\tau_z$','Interpreter',...
        'latex',"Location","eastoutside");
    set(lh, 'Interpreter', 'latex', 'fontsize', 14);
    title('')
    grid on
    sgtitle('Controller Plots', 'Interpreter','latex')
    if (choose.print.controller_plots)
       title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% Compare position optitrack data and EKF2 data
if (choose.plot.optitrakVsEKF2)
    title_str = 'Optitrack Vs EKF2';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    sub1 = subplot(3,1,1);
    hold on
    plot(plotdata.optitrack.Time, plotdata.optitrack.position.x,'--','LineWidth',display.opt.linewidth);
    plot(plotdata.estimated_states.p.Time  , plotdata.estimated_states.p.Data(:, 1),'LineWidth', display.opt.linewidth);
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('$x_o$', ...
        '$x_e$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sub2 = subplot(3,1,2);
    hold on
    plot(plotdata.optitrack.Time, plotdata.optitrack.position.y, '--', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.p.Time  , plotdata.estimated_states.p.Data(:, 2),'LineWidth', display.opt.linewidth);
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('$y_o$', ...
        '$y_e$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sub3 = subplot(3,1,3);
    hold on
    plot(plotdata.optitrack.Time, plotdata.optitrack.position.z, '--', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.p.Time  , plotdata.estimated_states.p.Data(:, 3),'LineWidth', display.opt.linewidth);
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('$z_o$', ...
        '$z_e$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    sgtitle(title_str, 'Interpreter','latex')
    display.pfig = [display.pfig fh];
%% Compare attitude optitrack data and EKF2 data
    title_str = 'Optitrack Vs EKF2';
    fh = figure('position', display.opt.position, 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    sub1 = subplot(3,1,1);
    hold on
    plot(plotdata.optitrack.Time, plotdata.optitrack.euler.x,'--','LineWidth',display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 1),'LineWidth', display.opt.linewidth);
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('$\phi_o$', ...
        '$\phi_e$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[deg]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sub2 = subplot(3,1,2);
    hold on
    plot(plotdata.optitrack.Time, plotdata.optitrack.euler.y, '--', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 2),'LineWidth', display.opt.linewidth);
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('$\theta_o$', ...
        '$\theta_e$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[deg]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sub3 = subplot(3,1,3);
    hold on
    plot(plotdata.optitrack.Time, plotdata.optitrack.euler.z, '--', 'LineWidth', display.opt.linewidth);
    plot(plotdata.estimated_states.euler.Time  , plotdata.estimated_states.euler.Data(:, 3),'LineWidth', display.opt.linewidth);
    hold off
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('$\psi_o$', ...
        '$\psi_e$', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
    xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[deg]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    sgtitle(title_str, 'Interpreter','latex')
    display.pfig = [display.pfig fh];
end
%% Arm joints Position tracking plot
if (settings.with_arm)
    if (choose.plot.arm_position_tracking)
        title_str = 'Arm Position Tracking';
        fh = figure('position', display.opt.position, 'name', title_str);
        set(gca, 'defaulttextinterpreter', 'latex');
        % set background to white
        set(gcf,'color','w');
        set(groot,'defaultAxesTickLabelInterpreter','latex');
        hold on
    %     plot(plotdata.references.theta_references.Time, plotdata.references.theta_references.Data(:, 1),'--','Color','#0072BD','LineWidth',display.opt.linewidth);
        plot(plotdata.references.theta_references.Time, plotdata.references.theta_references.Data(:, 2), '--','Color','#D95319', 'LineWidth', display.opt.linewidth);
        plot(plotdata.references.theta_references.Time, plotdata.references.theta_references.Data(:, 3), '--','Color','#EDB120', 'LineWidth', display.opt.linewidth);
    %     plot(plotdata.estimated_states.joints_states.Time  , plotdata.estimated_states.joints_states.position(:, 1),'Color','#0072BD','LineWidth', display.opt.linewidth);
        plot(plotdata.estimated_states.joints_states.Time  , plotdata.estimated_states.joints_states.position(:, 2),'Color','#D95319','LineWidth', display.opt.linewidth);
        plot(plotdata.estimated_states.joints_states.Time  , plotdata.estimated_states.joints_states.position(:, 3),'Color','#EDB120','LineWidth', display.opt.linewidth);
        highlight.ylimits = ylim;
        fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
            [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'r','LineStyle','none');
        alpha (fill1, highlight.tran);
    %     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
    %         [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'g','LineStyle','none');
    %     alpha (fill2, highlight.tran);
        hold off
        box on
        set(gca, 'FontSize', (display.opt.fontsize - 6))
        title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
        xlim([settings.plot_start,settings.plot_end])
        ylim([-2.5 2.5])
        lh = legend('$\theta_{r,1}$', '$\theta_{r,2}$', ... % '$\theta_{r,3}$', ...
            '$\theta_{1}$', '$\theta_{1}$', ... % '$p_{z}$', ...
            'Location', 'southeast', ...
            'Orientation', 'horizontal', 'interpreter', 'latex');
        xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
        ylabel('[rad]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
        set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
        grid on;
        display.pfig = [display.pfig fh];
        if (choose.print.arm_position_tracking)
            title_str = title_str(~isspace(title_str));
            file = [saving_path,'/',title_str,'','.pdf'];
            cmd = ['pdfcrop ',file,' ',file];
            set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
            set(gcf, 'PaperPositionMode', 'auto');
            print(gcf,'-dpdf',file);
            system(cmd);
        end
    end
    %% Arm joints torque tracking plot
    if (choose.plot.arm_torque_tracking)
        title_str = 'Arm Torque Tracking';
        fh = figure('position', display.opt.position, 'name', title_str);
        set(gca, 'defaulttextinterpreter', 'latex');
        % set background to white
        set(gcf,'color','w');
        set(groot,'defaultAxesTickLabelInterpreter','latex');
        hold on
    %     plot(plotdata.references.theta_references.Time, plotdata.references.theta_references.Data(:, 1),'--','Color','#0072BD','LineWidth',display.opt.linewidth);
        plot(plotdata.controller.arm_torque_commands.Time, plotdata.controller.arm_torque_commands.Data(:, 2), '--','Color','#D95319', 'LineWidth', display.opt.linewidth);
        plot(plotdata.controller.arm_torque_commands.Time, plotdata.controller.arm_torque_commands.Data(:, 3), '--','Color','#EDB120', 'LineWidth', display.opt.linewidth);
    %     plot(plotdata.estimated_states.joints_states.Time  , plotdata.estimated_states.joints_states.position(:, 1),'Color','#0072BD','LineWidth', display.opt.linewidth);
        plot(plotdata.estimated_states.joints_states.Time(1:100:end)  , plotdata.estimated_states.joints_states.effort(1:100:end, 2),'Color','#D95319','LineWidth', display.opt.linewidth);
        plot(plotdata.estimated_states.joints_states.Time(1:100:end)  , plotdata.estimated_states.joints_states.effort(1:100:end, 3),'Color','#EDB120','LineWidth', display.opt.linewidth);
        hold off
        box on
        set(gca, 'FontSize', (display.opt.fontsize - 6))
        title(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize);
        xlim([settings.plot_start,settings.plot_end])
        ylim([-6 6])
        lh = legend('$\tau_{r,1}$', '$\tau_{r,2}$', ... % '$\tau_{r,3}$', ...
            '$\tau_{1}$', '$\tau_{1}$', ... % '$\tau_{3}$', ...
            'Location', 'southeast', ...
            'Orientation', 'horizontal', 'interpreter', 'latex');
        xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
        ylabel('[N.m]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
        set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
        grid on;
        display.pfig = [display.pfig fh];
        if (choose.print.arm_torque_tracking)
            title_str = title_str(~isspace(title_str));
            file = [saving_path,'/',title_str,'','.pdf'];
            cmd = ['pdfcrop ',file,' ',file];
            set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
            set(gcf, 'PaperPositionMode', 'auto');
            print(gcf,'-dpdf',file);
            system(cmd);
        end
    end
end
%% battery status plot
if (choose.plot.battery_status)
    title_str = 'Battery status';
    fh = figure('position', [200 200 800 800], 'name', title_str);
    set(gca, 'defaulttextinterpreter', 'latex');
    % set background to white
    set(gcf,'color','w');
    sub1 = subplot(2,1,1);
    hold on
    plot(plotdata.battery.Time(1:2:end), plotdata.battery.voltage(1:2:end),'LineWidth',display.opt.linewidth);
    plot(plotdata.battery.Time(2:2:end), plotdata.battery.voltage(2:2:end),'LineWidth',display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('Battery 1', 'Battery 2', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
%     xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[v]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sub2 = subplot(2,1,2);
    hold on
    plot(plotdata.battery.Time(1:2:end), plotdata.battery.current(1:2:end),'LineWidth',display.opt.linewidth);
    plot(plotdata.battery.Time(2:2:end), plotdata.battery.current(2:2:end),'LineWidth',display.opt.linewidth);
    highlight.ylimits = ylim;
    fill1 = fill([highlight.t0 highlight.t0 highlight.t1 highlight.t1],...
        [highlight.ylimits(1)-4 highlight.ylimits(2)+4 highlight.ylimits(2)+4 highlight.ylimits(1)-4],'r','LineStyle','none');
    alpha (fill1, highlight.tran);
%     fill2 = fill([highlight.t1 highlight.t1 highlight.t2 highlight.t2],...
%         [highlight.ylimits(1)-2 highlight.ylimits(2)+2 highlight.ylimits(2)+2 highlight.ylimits(1)-2],'g','LineStyle','none');
%     alpha (fill2, highlight.tran);
    hold off
    box on
    set(gca, 'FontSize', (display.opt.fontsize - 6))
    xlim([settings.plot_start,settings.plot_end])
    lh = legend('Battery 1', 'Battery 2', ...
        'Location', 'southeast', ...
        'Orientation', 'horizontal', 'interpreter', 'latex');
%     xlabel('Time [s]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    ylabel('[A]', 'fontsize', display.opt.fontsize, 'interpreter', 'latex');
    set(lh, 'Interpreter', 'latex', 'fontsize', display.opt.fontsize);
    grid on;
    
    sgtitle(title_str, 'interpreter', 'latex', 'fontsize', display.opt.fontsize)
    display.pfig = [display.pfig fh];
    if (choose.print.attitude_tracking)
        title_str = title_str(~isspace(title_str));
        file = [saving_path,'/',title_str,'','.pdf'];
        cmd = ['pdfcrop ',file,' ',file];
        set(gcf,'PaperUnits','points', 'PaperSize', 1.1*Dimension );
        set(gcf, 'PaperPositionMode', 'auto');
        print(gcf,'-dpdf',file);
        system(cmd);
    end
end
%% End
disp('Finished Plotting.');
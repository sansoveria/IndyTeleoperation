function plot_single_demo(demo)
rod_length = 0.4*6;
rod_handle = zeros(size(demo.T_tool,1),4);
rod_end_effector = zeros(size(demo.T_tool,1),4);
stone = zeros(size(demo.T_stone,1),4);
cursor = zeros(size(demo.T_device,1),4);
for i = 1:size(demo.T_tool,1)
    rod_handle(i,:) = (squeeze(demo.T_tool(i,:,:))*[0;0;0;1])';
    rod_end_effector(i,:) = (squeeze(demo.T_tool(i,:,:))*[0;0;-rod_length;1])';
    stone(i,:) = (squeeze(demo.T_stone(i,:,:))*[0;0;0;1])';
    cursor(i,:) = (squeeze(demo.T_device(i,:,:))*[0;0;0;1])';
end

f = figure('Name', 'Demo', 'Position', [300, 500, 700, 400]);
ax1 = axes(f, 'OuterPosition', [0.0, 0.0, 0.7, 1.0]);

plot_range = 10:10:size(demo.T_tool,1);
hold(ax1, 'on');
plot_cursor = scatter3(ax1, cursor(plot_range,1), cursor(plot_range,2), cursor(plot_range,3), 1, 'b.');
plot_rod_effector = scatter3(ax1, rod_end_effector(plot_range,1), rod_end_effector(plot_range,2), rod_end_effector(plot_range,3), 1, 'r.');
plot_stone = scatter3(ax1, stone(plot_range,1), stone(plot_range,2), stone(plot_range,3), 1, 'k.');
% scatter3(cursor(:,1), cursor(:,2), cursor(:,3),'g.');
hold(ax1, 'off');

alpha(plot_cursor, 0.005);
alpha(plot_rod_effector, 0.005);
alpha(plot_stone, 0.005);

axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
view([133.58, 28.72]);
campos([14.17, 11.65, 10.17]);
camtarget([0.8, -1, 0.06]);

legend([plot_cursor, plot_rod_effector, plot_stone], ...
    {'User input', 'End effector', 'Stone'}, ...
    'AutoUpdate', 'off', ...
    'Location', 'north');

ax2 = axes(f, 'OuterPosition', [0.7, 0.0, 0.3, 1.0]);

contact_label = {'Support', 'Left rail', 'Right rail',...
    'Furnace enterance', 'Iron runner', 'Workspace nearfloor',...
    'Workspace farfloor', 'Molten iron', 'Furnace', 'Stone'};

for i = plot_range(1):30:plot_range(end)
    rod = [rod_handle(i,:); rod_end_effector(i,:)];

    hold(ax1, 'on');
    animation_cursor = plot3(ax1, cursor(i,1), cursor(i,2), cursor(i,3), '.b', 'MarkerSize', 30);  
    animation_rod = line(ax1, rod(:,1), rod(:,2), rod(:,3), 'Color', 'r' ,'LineWidth', 3);   
    animation_stone = plot3(ax1, stone(i,1), stone(i,2), stone(i,3), '.k', 'MarkerSize', 30);
    hold(ax1, 'off');
    
    barh(ax2, demo.contact_info(i, :));
    yticklabels(ax2, contact_label);
    xlim([0,1]);
        
    drawnow;
    pause(0.01);
    
    if i<plot_range(end)
        delete(animation_cursor);
        delete(animation_rod);
        delete(animation_stone);
    end
end
end

function plot_multiple_demos(ax, demo)
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

plot_range = 10:10:size(demo.T_tool,1);
hold(ax, 'on');
plot_cursor = scatter3(ax, cursor(plot_range,1), cursor(plot_range,2), cursor(plot_range,3), 1, 'b.');
plot_rod_effector = scatter3(ax, rod_end_effector(plot_range,1), rod_end_effector(plot_range,2), rod_end_effector(plot_range,3), 1, 'r.');
plot_stone = scatter3(ax, stone(plot_range,1), stone(plot_range,2), stone(plot_range,3), 1, 'k.');
% scatter3(cursor(:,1), cursor(:,2), cursor(:,3),'g.');
hold(ax, 'off');

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

end

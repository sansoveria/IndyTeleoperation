function demo = parse_log(folderName, fileIdx)

% folderName = "data";
fileList = dir(folderName);

% fileIdx = 1;
fileName = fileList(fileIdx+2).name;

data = dlmread(folderName+"/"+fileName);

data_length = size(data,1);
t = data(:,1);
T_device = zeros(data_length,4,4);
T_tool = zeros(data_length,4,4);
T_stone = zeros(data_length,4,4);
T_device(:,4,4) = 1;
T_device(:,1:3,4) = data(:,2:4);
T_tool(:,4,4) = 1;
T_tool(:,1:3,4) = data(:,8:10);
T_stone(:,4,4) = 1;
T_stone(:,1:3,4) = data(:,14:16);

for i = 1:data_length
    axis_device = data(i,5:7);
    angle_device = norm(axis_device);
    axang_device = [axis_device/angle_device, angle_device];
    T_device(i,1:3,1:3) = axang2rotm(axang_device);
    axis_tool = data(i,11:13);
    angle_tool = norm(axis_tool);
    axang_tool = [axis_tool/angle_tool, angle_tool];
    T_tool(i,1:3,1:3) = axang2rotm(axang_tool);
    axis_stone = data(i,17:19);
    angle_stone = norm(axis_stone);
    axang_stone = [axis_stone/angle_stone, angle_stone];
    T_stone(i,1:3,1:3) = axang2rotm(axang_stone);
end

force = data(:,20:22);
torque = data(:,23:25);

num_object = 10;
contact = zeros(data_length, num_object);
for i = 1:num_object
    contact(:,i) = bitand(data(:,26), 2^(i-1))>0;
end

demo = struct;
demo.t = t;
demo.T_device = T_device;
demo.T_tool = T_tool;
demo.T_stone = T_stone;
demo.contact_force = force;
demo.contact_torque = torque;
demo.contact_info = contact;

end
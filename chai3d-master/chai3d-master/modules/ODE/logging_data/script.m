clear all;
close all;

%% before 1218 (demonstration data collection)

folderName = "20201218 - data";


% fileIdx = 1;
% demo = parse_log(folderName, fileIdx);
% plot_single_demo(demo);

f = figure();
ax = gca;
for fileIdx = 1:50
    demo = parse_log(folderName, fileIdx);
    plot_multiple_demos(ax, demo);
end


%% after 1222 (feedback added)

folderName = "data";
demo = parse_log_long(folderName, 2);
plot_feedback(demo, [7,20]);


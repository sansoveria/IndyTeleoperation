function plot_feedback(demo, interval)
figure();

subplot(3,1,1);
plot(demo.t, demo.feedback(:,1), '-k', 'Linewidth', 1.5);
xlim(interval);
ylim([-3,3]);
xlabel('$$t$$ (s)','interpreter', 'latex');
ylabel('$$f_x$$ (N)','interpreter', 'latex');
set(gca,'LineWidth',1.5);
set(gca,'FontSize',15);
set(gca, 'fontname','Times New Roman');

subplot(3,1,2);
plot(demo.t, demo.feedback(:,2), '-k', 'Linewidth', 1.5);
xlim(interval);
ylim([-3,3]);
xlabel('$$t$$ (s)','interpreter', 'latex');
ylabel('$$f_y$$ (N)','interpreter', 'latex');
set(gca,'LineWidth',1.5);
set(gca,'FontSize',15);
set(gca, 'fontname','Times New Roman');

subplot(3,1,3);
plot(demo.t, demo.feedback(:,3), '-k', 'Linewidth', 1.5);
xlim(interval);
ylim([-3,3]);
xlabel('$$t$$ (s)','interpreter', 'latex');
ylabel('$$f_z$$ (N)','interpreter', 'latex');
set(gca,'LineWidth',1.5);
set(gca,'FontSize',15);
set(gca, 'fontname','Times New Roman');
end

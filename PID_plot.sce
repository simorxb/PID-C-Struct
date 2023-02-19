// Read from file
data_PID = [];
t_PID = [];
F_PID1 = [];
z_PID1 = [];
stp_PID1 = [];
F_PID2 = [];
z_PID2 = [];
stp_PID2 = [];

data_PID = read('PID C - Struct/data_PID_C.txt',-1,7);
t_PID = data_PID(:,1);
F_PID1 = data_PID(:,2);
z_PID1 = data_PID(:,3);
stp_PID1 = data_PID(:,4);
F_PID2 = data_PID(:,5);
z_PID2 = data_PID(:,6);
stp_PID2 = data_PID(:,7);

// Draw
subplot(212);
h = plot(t_PID, F_PID1, 'b-', t_PID, F_PID2, 'g-', 'LineWidth',3);
ax=gca(),// get the handle on the current axes
//ax.data_bounds=[0 -100;120 100];
set(gca(),"grid",[1 1]);
l = legend('Command PID1', 'Command PID2');
l.font_size = 3;
xlabel('t[s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('F [N]', 'font_style', 'times bold', 'font_size', 3);

subplot(211);
h = plot(t_PID, stp_PID1, 'r-', t_PID, stp_PID2, 'k-', t_PID, z_PID1, 'b-', t_PID, z_PID2, 'g-', 'LineWidth',3);
ax=gca(),// get the handle on the current axes
//ax.data_bounds=[0 0;120 250];
set(gca(),"grid",[1 1]);
l = legend('Setpoint object 1', 'Setpoint object 2', 'Response object 1', 'Response object 2');
l.font_size = 3;
xlabel('t[s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('z [m]', 'font_style', 'times bold', 'font_size', 3);

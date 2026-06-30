% Shanelle G. Clarke 
% fitting thrust stand data for sysid of bprl copter
close all 
clear all 


% find all data within a certain directory and load all 
% Specify the folder where your CSV files are located
folderPath = pwd + "/data/thrust_stand/"; % Use current directory or replace with specific path, e.g., 'C:\Users\YourUser\Documents\Data'

% 1. Read the CSV file
struct_data = load_data(folderPath,'csv');

total_data = [];

%% Data in file 

%  1. Time_s_
%  2. ESCSignal__s_
%  3. Servo1__s_
%  4. Servo2__s_
%  5. Servo3__s_
%  6. AccX_g_
%  7. AccY_g_
%  8. AccZ_g_
%  9. Torque_N_m_
% 10. Thrust_N_
% 11. Voltage_V_
% 12. Current_A_
% 13. MotorElectricalSpeed_
% 14. MotorOpticalSpeed_
% 15. ElectricalPower_W_
% 16. MechanicalPower_W_
% 17. MotorEfficiency_
% 18. PropellerMech_Efficiency_
% 19. OverallEfficiency_
% 20. Vibration_g_
% 21. AppMessage


 variable_array = [2,9,10,11,12,13,14];
 variable_names = {'PWM','Torque_N_m_','Thrust_N_','Voltage_V',...
                        'Current_A_','Mech_RPM','Optical_RPM'}; 

mean_data = zeros(size(struct_data.data{1}{:,variable_array}));
for i  = 1:int64(struct_data.num_files)
    data = [struct_data.data{i}{:,variable_array}];
    total_data = [total_data;data];
    mean_data = mean_data + data;
end
mean_data = mean_data/struct_data.num_files;


total_table = array2table(total_data,'VariableNames',variable_names);

mean_table = array2table(mean_data,'VariableNames',variable_names);


curveFitter

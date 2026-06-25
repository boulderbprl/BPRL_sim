% Shanelle G. Clarke 
% fitting thrust stand data for sysid of bprl copter


% find all data within a certain directory and load all 
% Specify the folder where your CSV files are located
folderPath = pwd + "/data/thrust_stand/"; % Use current directory or replace with specific path, e.g., 'C:\Users\YourUser\Documents\Data'

% 1. Read the CSV file
struct_data = load_data(folderPath,'csv');

total_data = [];
mean_data = zeros(size(struct_data.data{1}{:,[2,10,9,12,11]}));
for i  = 1:int64(struct_data.num_files)
    data = [struct_data.data{i}{:,[2,10,9,12,11]}];
    total_data = [total_data;data];
    mean_data = mean_data + data;
end
mean_data = mean_data/struct_data.num_files;
total_table = array2table(total_data,'VariableNames',{'PWM','Thrust','Torque','Current','Voltage'});

mean_table = array2table(mean_data,'VariableNames',{'PWM','Thrust','Torque','Current','Voltage'});

aggregated_curve_fit = curveFitter(total_table.PWM,total_table.Thrust); 
mean_curve_fit = curveFitter(mean_table.PWM,mean_table.Thrust); 

%aggregated_curve_fit_torque = curveFitter(total_table.PWM,total_table.Torque); 
%mean_curve_fit_torque = curveFitter(mean_table.PWM,mean_table.Torque); 

%aggregated_curve_fit_current = curveFitter(total_table.Current,total_table.Torque); 
%mean_curve_fit_current = curveFitter(mean_table.Current,mean_table.Torque); 

%aggregated_curve_fit_current = curveFitter(total_table.Current,total_table.Voltage); 
%mean_curve_fit_current = curveFitter(mean_table.Current,mean_table.Voltage); 
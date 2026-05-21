function struct_data = load_data(folderPath,ext)


%folderPath = pwd; % Use current directory, or specify 'C:\path\to\folder\'
filePattern = fullfile(folderPath, '**', "*."+ext); 

% 2. Get a list of matching files
files = dir(filePattern);

% 3. Initialize a cell array to store the loaded data
% A cell array is flexible for storing data of different types or sizes.
struct_data.num_files = numel(files); 
data = cell(numel(files), 1);

% 4. Loop through the file list and load each file
for i = 1:struct_data.num_files 
    % Construct the full file name and path
    fullFileName = fullfile(files(i).folder, files(i).name);
    
    % Display which file is being processed (optional)
    fprintf('Now processing %s\n', fullFileName);
    
    % create options object 
    opts = detectImportOptions(fullFileName);
    % The specific loading function depends on the file type:
    
    % Example for .txt files:
    % data{i} = readtable(fullFileName); 
    
    % Example for .csv files:
    data{i} = readtable(fullFileName,opts); % readtable automatically handles headers by default
    
end

struct_data.data = data; % Store the loaded data in the struct
struct_data.variables = opts.VariableNames; % Store the loaded data in the struct
end
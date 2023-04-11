% Implementation of a moving average filter for the given data.
% First, import data.
FILEPATH = "../../python_scripts/test11.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data;
N = max(size(y_data));
% Moving average filter takes the average of the last 5 samples
for i = 5:N
    y_data(i) = (y_data(i-1) + y_data(i-2) + y_data(i-3) + y_data(i-4) + y_data(i-5))/5;
end
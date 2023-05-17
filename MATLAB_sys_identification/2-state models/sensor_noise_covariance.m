FILEPATH = "../../python_scripts/Estimate_R.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data1';

R = cov(y_data)
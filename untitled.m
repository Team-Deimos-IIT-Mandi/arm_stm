clc;clear all;close all;
my_table = readtable('step_response_actuator_4.csv');

% Extract based on your specific column names
u = mydata.PWMInput;      % Your Input signal
y = mydata.SensorOutput;   % Your Output signal

% Force them into column vectors (the toolbox is picky about this)
u = u(:); 
y = y(:);
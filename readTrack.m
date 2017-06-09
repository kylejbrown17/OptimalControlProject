function [x,y,th,k,s,n] = readTrack()
%READTRACK Read raceway model file into arrays.
%   OUTPUTS:
%       x - (Nx1) world-frame x-coordinate
%       y - (Nx1) world-frame y-coordinate
%       th - (Nx1) world-frame heading (rad)
%       k - (Nx1) track curvature
%       s - (Nx1) track arclength
%       n - (Nx1) track half-width
%

fname = 'track.csv';

delimiter = ','; startRow = 2;
formatSpec = '%f%f%f%f%f%[^\n\r]';
fileID = fopen(fname,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, ...
    'HeaderLines' ,startRow-1, 'ReturnOnError', false);
fclose(fileID);

% Track parameters
x = dataArray{:, 1};
y = dataArray{:, 2};
th = dataArray{:, 3};
k = dataArray{:, 4};
s = dataArray{:, 5};
n = 30*zeros(size(s));

end

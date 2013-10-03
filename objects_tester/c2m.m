function [ output] = c2m( input )
%REPLACEEMPTY Summary of this function goes here
%   Detailed explanation goes here

for i=1:length(input)
    if(isequal(input(i),{''}))
        input(i)={'nan'};
    end
    str=cell2mat(input(i));
    output(i)=str2double(str);
end


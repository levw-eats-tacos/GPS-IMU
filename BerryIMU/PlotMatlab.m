fname = 'test.txt';
fid = fopen(fname); 
% open the file
% set linenum to the desired line number that you want to import
while linenum<=43
linenum = 1;
% use '%s' if you want to read in the entire line or use '%f' if you want to read only the first numeric value
C = textscan(fid,'%s',1,'delimiter','\n', 'headerlines',linenum-1)
fclose(fid); 
json_text=cellfun(@(x) x(1:end),C{:}, 'UniformOutput',false);
data=cellfun(@jsondecode,json_text,'UniformOutput',false)

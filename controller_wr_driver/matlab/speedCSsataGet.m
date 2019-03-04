clear all
global dat
delete(instrfind);
dat = serial('COM9', 'BaudRate', 115200);
dat.InputBufferSize = 4096;

fopen(dat)
set(dat, 'ByteOrder', 'littleEndian')

disp 'Ok!'

start = 'p';
fwrite(dat, start, 'uint8'); 

A = [];
B = [];
t = 10;

% for i = 1:t
%    A=fread(dat, [100,1], 'int16');
%    B = [B; A];
% end
 
start = 'a'; %forward
fwrite(dat, start, 'uint8'); 

for i = 1:t
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

start = 'd'; %backward
fwrite(dat, start, 'uint8'); 


for i = 1:2
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

start = 'd'; %backward
fwrite(dat, start, 'uint8'); 
for i = 1:t
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

start = 'a'; %forward
fwrite(dat, start, 'uint8'); 

fclose(dat);
disp 'Finish!'

% CNTRL = [];
% SPEED = []; 
% for i = 1: 2: (length(B) - 1)
%     CNTRL = [CNTRL; B(i, 1)];
% end
% CNTRL = CNTRL./100;
% for r = 2: 2:length(B)
%    SPEED = [SPEED; B(r, 1)]; 
% end
% 
% plot(CNTRL)
% grid on 
% hold on
% plot(SPEED)

plot(B)
grid on


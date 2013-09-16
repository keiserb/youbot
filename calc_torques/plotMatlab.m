clear act_pos act_vel ff_tor req_pos req_vel req_acc pd_tor old_tor new_tor int_tor t
close all
fid = fopen('values/act_pos.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    act_pos(i,1:5)=read(1:5,i);
    t(i)=(i-1)*0.02;
end
t=t';
fid = fopen('values/req_pos.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    req_pos(i,1:5)=read(1:5,i);
end

fid = fopen('values/act_vel.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    act_vel(i,1:5)=read(1:5,i);
end

fid = fopen('values/req_vel.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    req_vel(i,1:5)=read(1:5,i);
end

fid = fopen('values/req_acc.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    req_acc(i,1:5)=read(1:5,i);
end

fid = fopen('values/ff_tor.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    ff_tor(i,1:5)=read(1:5,i);
end

fid = fopen('values/pd_tor.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    pd_tor(i,1:5)=read(1:5,i);
end

fid = fopen('values/old_tor.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    old_tor(i,1:5)=read(1:5,i);
    t(i)=(i-1)*0.02;
end

fid = fopen('values/new_tor.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    new_tor(i,1:5)=read(1:5,i);
    t(i)=(i-1)*0.02;
end

fid = fopen('values/int_tor.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    int_tor(i,1:5)=read(1:5,i);
    t(i)=(i-1)*0.02;
end

for j=1:5
figure(j);
subplot(4, 1, 1);
hold on;
plot(t, req_pos(:,j));
plot(t, act_pos(:,j),'r');
xlabel('t');
ylabel('Position');
subplot(4, 1, 2);
hold on;
plot(t, req_vel(:,j));
plot(t, act_vel(:,j),'r');
xlabel('t');
ylabel('Velocity');
subplot(4, 1, 3);
hold on;
plot(t, req_acc(:,j));
xlabel('t');
ylabel('Acceleration');
subplot(4, 1, 4);
hold on;
plot(t, ff_tor(:,j), 'b');
plot(t, pd_tor(:,j),'r');
xlabel('t');
ylabel('Torque');
end

figure(6)
for j=1:5
subplot(5,1,j)
hold on
plot(t, ff_tor(:,j), 'b');
plot(t, pd_tor(:,j),'r');
plot(t, old_tor(:,j), 'g');
plot(t, new_tor(:,j), 'c');
plot(t, int_tor(:,j), 'k');
xlabel('t');
ylabel('Torque');
end


function IMAGINARY_WALL(data)

d=data;

data_array=table2array(d);
x=transpose(data_array(:,1));
y=transpose(data_array(:,2));
r=transpose(data_array(:,3));
theta=transpose(data_array(:,5));

%Setup upper and lower surfaces
x_t=[x;x];
y_t=[y;y];
z_t=[zeros(size(x));ones(size(x))];

%Setup the imaginary wall at 90% of the real physical walls
x_t_s=x_t*0.9;
y_t_s=y_t*0.9;
z_t_s=z_t/2;

%plot 2D plane of data
figure(1)
plot(x,y)
grid

%plot physical walls
figure(2)
surf(x_t,y_t,z_t);
hold on

%plot imaginary walls
surf(x_t_s,y_t_s,z_t_s);
hold off
grid on
end
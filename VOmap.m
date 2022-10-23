clear
clc

truth = readmatrix('kitti/poses/00.txt');

location = 0;
ox = 0;
oy = 0;
oz = 0;
for i = 1:size(truth,1)

    true_pose = [truth(i,1:4); truth(i,5:8); truth(i,9:12)];
    location = truth(i,4:4:12)';
    rotation = [truth(i,1:3); truth(i,5:7); truth(i,9:11)];
    x = location(1);
    y = location(2);
    z = location(3);

    % Create an xy-map
    plot([ox x],[oz z])
    title('Travel map')
    xlabel('x coordinate')
    ylabel('z coordinate')
    hold on

    % Old coordinates
    ox = x;
    oy = y;
    oz = z;

    if i == 27
    end

end
hold off
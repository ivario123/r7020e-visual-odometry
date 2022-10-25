function Error = PlotOnMap(Poses,end_index)
    p_size = size(Poses,1);
    truth = readmatrix('kitti/poses/00.txt');
    
    T_true = zeros(1,3);
    T_poses = zeros(1,3);
    Error = zeros(1,p_size);
    for i = 1:p_size
        cT_true = truth(i,4:4:12)';
        cT_poses = Poses(i).Translation;

        % Create an xz-map
        plot([T_true(1) cT_true(1)],[T_true(3) cT_true(3)],"Color",[0,0,0]);
        % Plot the pose data in the same frame
        plot([T_poses(1) cT_poses(1)],[T_poses(3) cT_poses(3)],"LineStyle","--");
        title('Travel map')
        xlabel('x coordinate')
        ylabel('z coordinate')
        hold on
        Error(i) = norm([cT_true(1) cT_true(3)]-[cT_poses(1) cT_poses(3)]);

        T_true = cT_true;
        T_poses = cT_poses;
    
    end
    hold off
end
function plot_line(points,data)
%plot_line: plots vertical lines at each point in points 
%   plots a vertical line at each point in points from the minimum to 
%   the maximum value in data alternating color for each point in points.
    if points(1) ~= 0
        for i=1:2:length(points)
            y = min(data):(max(data)-min(data))/100:max(data);
            x = zeros(1,length(y));
            x2 = zeros(1,length(y));
            x(:) = points(i);
            plot(x,y,'m.')
            hold on
            x2(:) = points(i+1);
            plot(x2,y,'y.')
        end
    end
end

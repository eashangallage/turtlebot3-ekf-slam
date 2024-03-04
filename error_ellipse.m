clear all;
close all;

data_el = csvread("plot_log.csv");
num_lines = length(data_el);

data_tru = csvread("nusim_log.csv");
data_tru_x = data_tru(:,2)*10;
data_tru_y = data_tru(:,3)*10;
hold on;

plot(data_tru_x,data_tru_y,'r',LineWidth=2)

data_odom = csvread("odom_log.csv");
data_odom_x = data_odom(:,2)*10;
data_odom_y = data_odom(:,3)*10;

plot(data_odom_x,data_odom_y,'bl',LineWidth=2)

data_slam = csvread("slam_log.csv");
data_slam_x = data_slam(:,2)*10;
data_slam_y = data_slam(:,3)*10;

plot(data_slam_x,data_slam_y,'g',LineWidth=2);

data_el(:,2) = data_slam_x;
data_el(:,3) = data_slam_y;

for i = 1: num_lines
    plot_sllipse(data_el(i,:))
end

legend({'True Trajectory','Odom/ Predicted Trajectory','SLAM/ Corrected Trajectory'},'Location','southwest')
xlabel('X(m)') 
ylabel('Y(m)') 

function f = plot_sllipse(n)
    
    % Calculate the eigenvectors and eigenvalues
    covx = [n(4), n(5), n(6); n(7), n(8), n(9);n(10), n(11), n(12)];
    [xeigenvec, xeigenval ] = eig(covx);

    % Get the coordinates of the data mean
    xavg = [n(2) n(3)];
    
    % Get the index of the largest eigenvector    
    [xlargest_eigenvec_ind_c, xr] = find(xeigenval == max(max(xeigenval)));
    xlargest_eigenvec = xeigenvec(:, xlargest_eigenvec_ind_c);
    
    % Get the largest eigenvalue    
    xlargest_eigenval = max(max(xeigenval));
    
    % Get the smallest eigenvector and eigenvalue
    if(xlargest_eigenvec_ind_c == 1)    
        xsmallest_eigenval = max(xeigenval(:,2));
        xsmallest_eigenvec = xeigenvec(:,2);
    else
        xsmallest_eigenval = max(xeigenval(:,1));
        xsmallest_eigenvec = xeigenvec(:,1);
    end
    
    % Calculate the angle between the x-axis and the largest eigenvector
    xangle = atan2(xlargest_eigenvec(2), xlargest_eigenvec(1));
    
    % This angle is between -pi and pi.
    % Let's shift it such that the angle is between 0 and 2pi    
    if(xangle < 0)
        xangle = xangle + 2*pi;
    end
    
    % Get the 95% confidence interval error ellipse
    % chisquare_val = 2.4477;
    chisquare_val = 0.5;
    theta_grid = linspace(0,2*pi);
    
    phi = xangle;
    xX0=xavg(1);
    xY0=xavg(2);
    xa=chisquare_val*sqrt(xlargest_eigenval)/10;
    xb=chisquare_val*sqrt(xsmallest_eigenval)/10;
    
    % the ellipse in x and y coordinates 
    ellipse_x_r  = xa*cos( theta_grid );
    ellipse_y_r  = xb*sin( theta_grid );
    
    %Define a rotation matrix
    R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
    
    %let's rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
    
    % Draw the error ellipse
    plot(r_ellipse(:,1) + xX0,r_ellipse(:,2) + xY0,'-','Color', [0.5, 0.25, 0])
    hold on;
    
    quiver(xX0, xY0, xlargest_eigenvec(1), xlargest_eigenvec(2), '-m', 'LineWidth',1);
    quiver(xX0, xY0, xsmallest_eigenvec(1), xsmallest_eigenvec(2), '-g', 'LineWidth',1);
    hold on;
    
    % Set the axis labels
    hXLabel = xlabel('x');
    hYLabel = ylabel('y');
end
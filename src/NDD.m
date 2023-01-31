function[ img ] = NDD( ptcloud,num_sector,num_ring,max_range )
%% Downsampling
% pcshow(ptcloud);
gridStep = 0.75; % cubic grid downsampling(m). 
ptcloud = pcdownsample(ptcloud, 'gridAverage', gridStep);

%% pca
 [V,~,eig] = pca(ptcloud.Location);
if eig(2,:)/3 > eig(3,:)
    fd= V(:,3);
elseif eig(1,:)/3 > eig(2,:)
    fd= V(:,1);
else
    fd = [1,0,0];
end
R = [V(:,1),V(:,2),cross(V(:,1),V(:,2))];
% R = [cross(V(:,3),V(:,2)),V(:,2),V(:,3)];
fd = R*fd;%feature direction
yaw = atan( fd(2,:)/fd(1,:))+pi/2;
A = [cos(yaw) sin(yaw) 0 ; ...
    -sin(yaw) cos(yaw) 0 ; ...
    0 0 1 ];
ptcloud_tran =ptcloud.Location*A;

%% Save a point to the corresponding bin 
angle_one_sector = 360/num_sector;
[theta,r] = cart2pol(ptcloud_tran(:,1),ptcloud_tran(:,2));
theta = theta + pi;

r_step = max_range / num_ring;
r_index = ceil(r/r_step);
r_index(r_index>20) = 20;

s_step = angle_one_sector*pi/180;
theta(theta>2*pi-eps) = 0;
s_index = ceil(theta/s_step);
s_index(s_index<1) = 1;

img = zeros(num_ring, num_sector,2);

for i=1:num_ring
    for j=1:num_sector
        idx = r_index== i & s_index==j;
        point = ptcloud_tran(idx,:);
        if size(point,1) > 4
             points_in_bin_mean = mean(point);
             points_in_bin_cov = cov(point);
                [U,S,V] = svd(points_in_bin_cov);
                if S(3,3) < 0.001 * S(1,1)
                    S(3,3) = 0.001 * S(1,1);
                    points_in_bin_cov = U*S*V';
                end
                  [~, posDef] = chol(points_in_bin_cov);                
                if posDef ~= 0
                    % If the covariance matrix is not positive definite,
                    % disregard the contributions of this cell.
                    continue;
                end
                %entropy
                N = 4;
                entropy = (N/2)*(1+log(2*pi)) + 0.5*log(det(points_in_bin_cov));% log
%                 points_in_bin_covinv = inv(points_in_bin_cov);
                q = point - points_in_bin_mean; 
                gaussianValue = -q / points_in_bin_cov * q'/2;
                score = sum(exp(diag(gaussianValue)));
        else
            entropy = 0;
            score = 0;
        end
        img(i, j, 1)= entropy;
        img(i, j, 2)= score;
    end
end
end

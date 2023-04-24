function[ img ] = INDD( ptcloud,num_sector,num_ring,max_range )
%% Downsampling
% tic
gridStep = 0.75; % cubic grid downsampling(m). 
ptcloud = pcdownsample(ptcloud, 'gridAverage', gridStep);
% toc

%% ROTATION INVARIANCE TRAN 
% tic
 [V,~,latent] = pca(ptcloud.Location);
if latent(2,:)/3 > latent(3,:)
    fd= V(:,3);
elseif latent(1,:)/3 > latent(2,:)
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

angle_one_sector = 360/num_sector;
% toc

%% Save a point to the corresponding bin
% tic
% to polar coordinates
[theta,r] = cart2pol(ptcloud_tran(:,1),ptcloud_tran(:,2));

% so that theta in [0 2*pi]
theta(theta<0) = 2*pi+theta(theta<0);

% Intensity calibration and normalization
% I_c = ptcloud.Intensity .* r.^2 / 40;

% r step and r index for one point
r_step = max_range / num_ring;
r_index = ceil(r/r_step);
r_index(r_index>20) = 20;

% s step and s index for one point
s_step = angle_one_sector*pi/180;
theta(theta>2*pi-eps) = 0;
s_index = ceil(theta/s_step);
s_index(s_index<1) = 1;
% toc
%% bin to image format (2D matrix) 
% tic
img = zeros(num_ring, num_sector,2);

N = 3;
cst = (N/2)*(1+log(2*pi)); % a constant scaler

% loop for one bin
for i=1:num_ring
    for j=1:num_sector

        % index of points in that bin
        idx = ( r_index==i & s_index==j );
        
        % points in that bin, [x,y,z,I_c]
%         points_in_bin_ij = [ptcloud_tran(idx,:) ,I_c(idx,:)];
        points_in_bin_ij = ptcloud_tran(idx,:);
        % if less than 5 points in a bin, reject this bin
        if size(points_in_bin_ij,1)<5
            img(i,j,1) = 0;
            img(i,j,2) = 0;
            continue;
        end

        % mean and cov
        points_in_bin_mean = mean(points_in_bin_ij);
        points_in_bin_cov = cov(points_in_bin_ij);

        q = points_in_bin_ij - points_in_bin_mean; % points with mean 0
        
        %%
        % score
        % we should use eig but we found that eig and svd may result in different results
%         [Ve, Se] = eig(points_in_bin_cov); 
        [V,S,~] = svd(points_in_bin_cov);
        if S(1,1)<1e-5 || S(2,2)<1e-5
            % If the covariance matrix is not positive definite,
            % disregard the contributions of this cell.
            continue;
        end
        
        % ensure positive definite
        if S(3,3) < 0.001 * S(1,1)
            S(3,3) = 0.001 * S(1,1);
        end
%         if S(4,4) < 0.001 * S(1,1)
%             S(4,4) = 0.001 * S(1,1);
%         end

        L = bsxfun( @times, V, sqrt(1./diag(S)') );
        A = q * L;
        score = sum( exp( -sum(A.^2,2)/2 ) );

        % entropy
%         entropy = cst + 0.5*log(det(points_in_bin_cov));% log
        entropy = cst + 0.5*log(prod(diag(S)));% log


        img(i,j,1) = entropy;
        img(i,j,2) = score;

    end
end
% toc


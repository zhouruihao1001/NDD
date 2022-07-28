function[ D ] = NDD( ptcloud,N_sector,N_ring,range )
%% Downsampling
gridStep = 0.75; % uniform downsampling(m). 
ptcloud = pcdownsample(ptcloud, 'gridAverage', gridStep);
% pcshow(ptcloud);

%% PCA
 [V,~,eig] = pca(ptcloud.Location);
if eig(2,:)/3 > eig(3,:)
    fd= V(:,3);
elseif eig(1,:)/3 > eig(2,:)
    fd= V(:,1);
else
    fd = [1,0,0];
end
R = [V(:,1),V(:,2),cross(V(:,1),V(:,2))];

fd = R*fd;%feature direction
yaw = atan( fd(2,:)/fd(1,:))+pi/2;
A = [cos(yaw) sin(yaw) 0 ; ...
    -sin(yaw) cos(yaw) 0 ; ...
    0 0 1 ];
ptcloud_rot =ptcloud.Location*A;


%% cell bin
num_points = ptcloud.Count;
% num_points = length(ptcloud);
gap = range / N_ring; 
angle_one_sector = 360/N_sector;

cell_bins = cell(N_ring, N_sector);
cell_bin_counter = ones(N_ring, N_sector);

enough_large = 500; % max points in cell
enough_small = -1000;

for ith_ring = 1:N_ring
   for ith_sector = 1:N_sector
        bin_ij = enough_small * ones(enough_large, 3);
        cell_bins{ith_ring, ith_sector} = bin_ij;
   end
end
%% Cell Dividing
for ith_point =1:num_points

    % Point information
    ith_point_xyz = ptcloud_rot(ith_point,:);
    ith_point_r = sqrt(ith_point_xyz(1)^2 + ith_point_xyz(2)^2);
    ith_point_theta = XY2Theta(ith_point_xyz(1), ith_point_xyz(2)); % degree
    
    % Find the corresponding ring index 
    tmp_ring_index = floor(ith_point_r/gap);
    if(tmp_ring_index >= N_ring)
        ring_index = N_ring;
    else
        ring_index = tmp_ring_index + 1;
    end
    
    % Find the corresponding sector index 
    tmp_sector_index = ceil(ith_point_theta/angle_one_sector);
    if(tmp_sector_index == 0)
        sector_index = 1;
    elseif(tmp_sector_index > N_sector || tmp_sector_index < 1)
        sector_index = N_sector;
    else
        sector_index = tmp_sector_index;
    end
    
    % Assign point to the corresponding bin cell 
    try
        corresponding_counter = cell_bin_counter(ring_index, sector_index); % 1D real value.
    catch
        continue;
    end
    cell_bins{ring_index, sector_index}(corresponding_counter, :) = ith_point_xyz;
    cell_bin_counter(ring_index, sector_index) = cell_bin_counter(ring_index, sector_index) + 1; % increase count 1
    
end
%% Descriptor
D = zeros(N_ring, N_sector,2);

N = 3;
min_num_thres = 5; %  noise-filter para.

% Description
for ith_ring = 1:N_ring
    for ith_sector = 1:N_sector
        points_in_cell = cell_bins{ith_ring, ith_sector};
        if( num_minpoints(points_in_cell, min_num_thres, enough_small) )
            for i = 1:enough_large
                if(points_in_cell(i,1) == enough_small) 
                    num_incell = i-1;
                    break;
                end                
            end
                points_in_cell = points_in_cell(1:num_incell,:);
                %Normal Distribution
                points_in_cell_mean = mean(points_in_cell);
                points_in_cell_cov = cov(points_in_cell);
                %Prevent covariance matrix from going singular (and not be invertible)
                [U,S,V] = svd(points_in_cell_cov);
                if S(3,3) < 0.001 * S(1,1)
                    S(3,3) = 0.001 * S(1,1);
                    points_in_cell_cov = U*S*V';
                end
                  [~, posDef] = chol(points_in_cell_cov);                
                if posDef ~= 0
                    % If the covariance matrix is not positive definite,
                    % disregard the contributions of this cell.
                    continue;
                end
               
                %Entropy
                entropy = (N/2)*(1+log(2*pi)) + 0.5*log(det(points_in_cell_cov));% ln
                % Probability Density
                q = points_in_cell - points_in_cell_mean; 
                gaussianValue =exp( -q / points_in_cell_cov * q'/2);
                score = trace(gaussianValue);     
        else
            entropy = 0;
            score = 0;
        end
        D(ith_ring, ith_sector,1)= score;
        D(ith_ring, ith_sector,2)= entropy;  

    end
end


end


%%
function bool = num_minpoints(mat, minimum_thres, enough_small)

min_thres_point = mat(minimum_thres, :);

if( isequal(min_thres_point, [enough_small, enough_small, enough_small]) )
    bool = 0;
else
    bool = 1;
end

end
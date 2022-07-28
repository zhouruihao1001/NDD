function [corr_similarity] = corr_sim_P(ndd1,ndd2)

num_rings = size(ndd1, 1);
num_sectors = size(ndd1, 2);
geo_sim = zeros(1, num_sectors);

row_vec2= sum(ndd2);

for i = 1:num_sectors
    one_step = 1; % const 
    ndd1 = circshift(ndd1, one_step, 2); 
    row_vec1 = sum(ndd1);
    geo_sim(i) = dot(row_vec1, row_vec2) / (norm(row_vec1)*norm(row_vec2));
end  
    

    [~,shiftcol] = max(geo_sim);
    ndd1 = circshift(ndd1, shiftcol, 2); % 2 means columne shift 
    
%% sim calc.
    a= ndd1;
    b= ndd2;
    a = a - mean2(a);
    b = b - mean2(b);
    corr_similarity = sum(sum(a.*b))/sqrt(sum(sum(a.*a))*sum(sum(b.*b)));
  

end

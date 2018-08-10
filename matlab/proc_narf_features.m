i1 = 1;
x1 = 2;
y1 = 3;
z1 = 4;
i2 = 5; 
x2 = 6;
y2 = 7;
z2 = 8;
dist = 9;
desc_dist = 10;

total_feat_1 = 0;
total_feat_2 = 0;
dists_compare = [0:0.1:2, 3:1:20, 40, 60];
feat_dists_1 = zeros(size(dists_compare));
desc_dists_1 = zeros(size(dists_compare));
feat_dists_2 = zeros(size(dists_compare));
    
for i=1:1000
    filename1 = strcat('/home/boroson/data/kitti/features/narf/', num2str(i), '_keypt_1.csv');
    cloud1_matches = csvread(filename1, 1, 0);
    
    filename2 = strcat('/home/boroson/data/kitti/features/narf/', num2str(i), '_keypt_2.csv');
    cloud2_matches = csvread(filename2, 1, 0);
    
    for j=1:size(cloud1_matches, 1)
        feat_dist = sqrt(cloud1_matches(j,dist));
        
        dist_flags = feat_dist < dists_compare;
        feat_dists_1(dist_flags) = feat_dists_1(dist_flags) + 1;
        desc_dists_1(dist_flags) = desc_dists_1(dist_flags) + cloud1_matches(j,desc_dist);
    end
    
    for j=1:size(cloud2_matches,1)
        feat_dist = sqrt(cloud2_matches(j,dist));
        
        dist_flags = feat_dist < dists_compare;
        feat_dists_2(dist_flags) = feat_dists_2(dist_flags) + 1;
    end
end

figure; plot(dists_compare, feat_dists_1/feat_dists_1(end), dists_compare, feat_dists_2/feat_dists_2(end))
figure; plot(dists_compare, desc_dists_1./feat_dists_1)
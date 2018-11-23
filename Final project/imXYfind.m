function [x,y]=imXYfind(cam)
I = snapshot(cam);
I = I(:,:,1);
% figure(1)
% subplot(3,1,1)
% imshow(I);
BW = imtophat(I,strel('square',55));
BW = imbinarize(BW);
% subplot(3,1,2)
% imshow(BW)
cc = bwconncomp(BW,4);
c2 = regionprops(cc,'basic')
cent = [c2.Centroid]
ars = [c2.Area];
k = 0
s = size(ars)
idx = 1
for i = 1:s(2)
    if ars(i)>=500 && ars(i) <=2000 
        square = false(size(BW));
        square(cc.PixelIdxList{i}) = true;
        idx = i
        k = k+1;
        j = k+1
        figure(j)
        imshow(square)
        break
    end
end
[r,c] = find(square ==1);
x = round(mean(r))
y = round(mean(c))
end
function edit_map(input_filename, output_filename)
disp('Loading Map');
data = load(input_filename);
disp('Map Loaded');

res = data(1);
map = reshape(data(4:end), data(2), data(3));

figure(1);
imagesc(map)
colormap(gray)
axis image;

disp('Crop the image leaving extra room at the boundaries for rotation.');
disp('Double Click to Complete');
map_crop = imcrop();
clf;

disp('rotate to align the image');
map_rot = rotationGUI(map_crop, 50);
clf;

disp('Make the final crop');
disp('Double Click to Complete');    
imagesc(map_rot)
colormap(gray)
axis image;

final_map = imcrop();

imagesc(final_map)
colormap(gray)
axis image;

disp('Select the origin');
[y x] = ginput(1);
disp(['Origin (from corner) : ' num2str(x) ', ' num2str(y)]);
disp(['Origin (from center) : ' ...
    num2str(x - (size(final_map,1)/2)) ', ' ...
    num2str(y - (size(final_map,2)/2))]);

hold on
plot([y y], [x x+10], 'r-', 'LineWidth', 2);
plot([y y+10], [x x], 'g-', 'LineWidth', 2);
hold off


disp('Saving Final Map');
output = [res; size(final_map)'; final_map(:)]';

%save(output_filename,'output','-ascii');
dlmwrite(output_filename, output, 'delimiter', ' ', 'precision', '%10.8f');

end
    
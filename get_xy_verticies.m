function xy_error_verticies = get_xy_verticies(x_error_set,y_error_set)
%{
Generate 2D box uncertianity sets given 2 1D uncertanity sets.

:vector x_error_set: uncertanity in x-axis
:vector y_error_set: uncertanity in y-axis
:matrix return: 2d uncertianiaty set described by 4 verticies
%}
xy_error_verticies=zeros(2,4);
for i=1:2
    for j=1:2
        xy_error_verticies(:,j+(i-1)*2)=[x_error_set(i);y_error_set(j)];
    end
end
end
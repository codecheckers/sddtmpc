angle=20;
i=(-(angle/2):0.5:angle)*pi/180;
original=Polyhedron([a,0;0,b;-a,0;0,-b]);
old=original;
y=zeros(length(i),1);
k=1;
hold on


for j=i
    original=original&getPolyhedrong(j,a,p);
end
plot(getPolyhedrong(angle*pi/180,a,p));
plot(original,'color','black')
plot(old*(1/(cos(angle*pi/180)+sin(angle*pi/180))),'color','blue')
legend('set U if error in direction is equal to 0.3473 rad',...
    'set U if error in direction is between -0.3473 to 0.3473 rad','\lambda(0.3473)*U')

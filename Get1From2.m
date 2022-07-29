function [xout,yout]=Get1From2(x1,y1,x2,y2,l1,l2,limit)
% get a point position with its distances to two ref points
% this function will return 99 if no solution found
% limit: yBigger, ySmaller, xBigger, xSmaller

D=sqrt((x1-x2)^2+(y1-y2)^2);
if ~(l1+l2>=D && abs(l1-l2)<D)
    xout=99;
    yout=99;
    return;
end
% if abs(l1+l2-D)<10^-10
%     xout=x1+l1/D*(x2-x1);
%     yout=y1+l1/D*(y2-y1);
%     return;
% end
A=(l1+l2)^2-(x1-x2)^2-(y1-y2)^2;
B=-(l1-l2)^2+(x1-x2)^2+(y1-y2)^2;
E=(-l1^2+l2^2)*(y1-y2)+(y1+y2)*((x1-x2)^2-y1*y2)+y1^3+y2^3;
F=2*((x1-x2)^2+(y1-y2)^2);
temp=sqrt(A*B);
yo1=(x1*temp-x2*temp+E)/F;
yo2=(x2*temp-x1*temp+E)/F;
xo1=sqrt(l1^2-(yo1-y1)^2)+x1;
if abs((xo1-x2)^2+(yo1-y2)^2-l2^2)>10^-4
    xo1=-sqrt(l1^2-(yo1-y1)^2)+x1;
end
xo2=sqrt(l1^2-(yo2-y1)^2)+x1;
if abs((xo2-x2)^2+(yo2-y2)^2-l2^2)>10^-10
    xo2=-sqrt(l1^2-(yo2-y1)^2)+x1;
end
if abs(yo1-yo2)<10^-4
    xo2=2*x1-xo1;
end
if strcmp('yBigger',limit)
    if yo1>yo2
        yout=yo1;
        xout=xo1;
    else
        yout=yo2;
        xout=xo2;
    end
elseif strcmp('ySmaller',limit)
    if yo1<yo2
        yout=yo1;
        xout=xo1;
    else
        yout=yo2;
        xout=xo2;
    end
elseif strcmp('xSmaller',limit)
    if xo1<xo2
        yout=yo1;
        xout=xo1;
    else
        yout=yo2;
        xout=xo2;
    end
elseif strcmp('xBigger',limit)
    if xo1>xo2
        yout=yo1;
        xout=xo1;
    else
        yout=yo2;
        xout=xo2;
    end
else
    error('wrong input option!');
end
end
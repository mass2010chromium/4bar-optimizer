% function retval = circleIntersection(o1, r1, o2, r2)
%   vNormal=[0,0,1];
%   O=o2-o1;
%   cc=norm(O);
%   o1_theta=acos((-r2 ^ 2 + r1 ^ 2 + cc ^ 2)/(2*cc*r1));
%     % cos(theta)=(C^2-A^2-B^2)/2AB
%   r=r1*sin(o1_theta);
%   rhat=unit(cross(vNormal,O)); %TODO FIX TO MAKE MORE SENSE
%   mid=o1+unit(O)*r1*cos(o1_theta);
%   R=rhat*r;
%   retval={mid+R,mid-R};
%   return;
% end

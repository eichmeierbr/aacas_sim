classdef Obstacle < Objects
   properties
       
   end
    
   methods
       function obj = Obstacle(pos, vel, rad)
            obj.rad = 0.254;
            obj.numSides = 100;
            obj.color = 'red';
          if nargin > 0
              obj.state(1:3) = pos;
          end
          if nargin >1
              obj.state(4:6) = vel;
          end
          if nargin > 2
              obj.rad = rad;
          end
          obj.state_hist = [obj.state];
       end

   end
end
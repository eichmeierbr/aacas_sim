classdef (Abstract) MotionController < handle
    % Abstract class to implement motion control for the Object class
    % This class only has a move method.
    % Move returns a 12x1 array for the xdot [pos,vel,orient,rot].
   methods(Abstract)
       move(obj,inState)
   end
end
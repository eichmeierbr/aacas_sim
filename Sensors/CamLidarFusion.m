classdef CamLidarFusion < handle
    properties
        vision
        lidar
    end
	
    methods
        function obj = CamLidarFusion()
           obj.vision = VisionSensor;
           obj.lidar = RangeSensor;
        end
        
         function [xo, yo, dist_o] = getObstacleDetections(obj,q, th, world)
            [xo, yo, dist_o] = obj.vision.getObstacleDetections(q, th, world);
            obj.vision.plotMeasurements(q,xo,yo);
            [xo2, yo2, dist_o2] = obj.lidar.getObstacleDetections(q, th, world);
            obj.lidar.plotMeasurements(q,xo2,yo2);
            
            for i=1:max(size(xo2))
                if xo2(i) ~= Inf
                    xo = [xo; xo2(i)];
                    yo = [yo; yo2(i)];
                    dist_o = [dist_o; dist_o2(i)];
                end
            end
         end 
         
         function plotMeasurements(obj, q, xo, yo)
            obj.vision.plotMeasurements(q,xo,yo);
            obj.lidar.plotMeasurements(q,xo,yo);
         end
    end
end
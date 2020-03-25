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
            [xo, yo, dist_o] = obj.lidar.getObstacleDetections(q, th, world);
            obj.lidar.plotMeasurements(q,xo,yo);
         end 
         
         function plotMeasurements(obj, q, xo, yo)
            obj.vision.plotMeasurements(q,xo,yo);
            obj.lidar.plotMeasurements(q,xo,yo);
         end
    end
end
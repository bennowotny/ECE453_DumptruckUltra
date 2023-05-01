import cv2 as cv2
import numpy as np
import math

class DistanceSampler2:

    def __init__(self, pixelHeight, knownDistance, absoluteWidth, absoluteHeight):
        self.absoluteWidth = absoluteWidth
        self.absoluteHeight = absoluteHeight
        self.focalLength = (pixelHeight*knownDistance)/absoluteHeight

    def computeDistance(self, x1, y1, x2, y2):
        d = math.sqrt((x2-x1)**2+(y2-y1)**2)
        return d

    def getDistance(self, bbox_height):
        if bbox_height>0:
            return (self.absoluteHeight)*(self.focalLength)/bbox_height
        else:
            return 0


    def getAngle(self, x_coord, y_coord, bbox_width, bbox_height,width,height):
        # angle = 2*math.atan(bbox_height/(2*self.focalLength))
        horizontal_angle = math.atan2(x_coord+bbox_width/2-width/2, self.focalLength)
        # vertical_angle = math.atan2(y_coord+bbox_height/2-height/2, self.focalLength)
        # horizontal_angle = math.degrees(horizontal_angle)
        # vertical_angle = math.degrees(vertical_angle)
        # print(f'Horizontal and vertical angles {horizontal_angle,vertical_angle}')
        return horizontal_angle

    def getRealXYFlatPlane(self, horizontal_angle, distance):
        real_x_dist_flat = math.sin(horizontal_angle)*distance
        real_y_dist_flat = math.cos(horizontal_angle)*distance
        return (real_x_dist_flat, real_y_dist_flat)

    def getRealXY(self, angle, distance, y_coord, img_height):
        real_x_dist = math.tan(angle)*distance
        real_y_dist = (y_coord-(img_height/2))*(distance/self.focalLength)
        return (real_x_dist,real_y_dist)

    def getFocalLength(self):
        return self.focalLength



class DistanceSampler:

    def __init__(self, focalLength, knownDistance, absoluteWidth, absoluteHeight):
        self.absoluteWidth = absoluteWidth
        self.absoluteHeight = absoluteHeight
        self.focalLength = focalLength
        self.pixelHeight = (self.focalLength*self.absoluteHeight)/knownDistance
        self.pixelWidth = (self.focalLength*self.absoluteWidth)/knownDistance

    def computeDistance(self, x1, y1, x2, y2):
        d = math.sqrt((x2-x1)**2+(y2-y1)**2)
        return d

    def getDistance(self, bbox_height):
        # We use height because there is a greater proportion between the height and the frame
        distance = (self.absoluteHeight*self.focalLength)/(self.pixelHeight*bbox_height)
        return distance

    def getAngle(self, distance):
        angle = math.atan((0.5*self.absoluteHeight)/distance)*2

    def getFocalLength(self):
        return self.focalLength

if __name__=='__main__':
    pixel_size = 398.0      #3.98 um
    focalLength = 3.67     # in mm
    knownDistance = 50      # in mm
    absoluteWidth = 49.784  # 1.96 in
    absoluteHeight = 49.784 # 1.96 in

    bbox_height = 382
    # Run some tests given our model's bbox measurements
    distanceSampler = DistanceSampler(focalLength, knownDistance, absoluteWidth, absoluteHeight)
    distanceSampler2 = DistanceSampler2(
            float(pixel_size),
            float(knownDistance),
            float(absoluteWidth),
            float(absoluteHeight)
    )

    print(f'First: {distanceSampler.getDistance(bbox_height)}')
    print(f'Second: {distanceSampler2.getDistance(bbox_height)}')
    print(f'Second focal length: {distanceSampler2.getFocalLength()}')

import sys
import numpy as np
import cv2

class BlockVision:

    granularity = 2
    showThreshold = False
    debugColors = False
    blocks = {
        'purple':[],
        'orange':[],
        'green':[],
        'blue':[],
        'red':[],
        'yellow':[]
    }
    colors = {
        'purple': {'upper': np.array([175,255,255]), 'lower': np.array([163,50,50])},
        'orange': {'upper': np.array([10,255,255]), 'lower': np.array([4,50,50])},
        'green': {'upper': np.array([102,255,255]), 'lower': np.array([63,20,20])},
        'blue': {'upper': np.array([130,255,255]), 'lower': np.array([110, 30, 30])},
        'red': {'upper': np.array([3,255,255]), 'lower': np.array([0,30,30])},
        'red2': {'upper': np.array([180,255,255]), 'lower': np.array([177,30,30])},
        'yellow': {'upper': np.array([30,255,255]), 'lower': np.array([24,32,20])}
    }

    def __init__(self, img):
        self.image = img
        if self.image is None:
            print 'Failed to load image file:', imgPath
            sys.exit(1)

    def findAllBlocks(self):
        for color, blocks in self.blocks.iteritems():
            self.findBlocks(color)

    def drawAllCenters(self):
        for color, blocks in self.blocks.iteritems():
            self.drawCenters(color)

    def findBlocks(self,color):
        self.height, self.width, c = self.image.shape

        #convert the image to HSV space
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        self.image_hsv = hsv

        #define a range for color
        lower = self.colors[color]['lower']
        upper = self.colors[color]['upper']

        #threshold for color
        color_img = cv2.inRange(hsv,lower,upper)

        #blur the image
        color_img = cv2.GaussianBlur(color_img,(9,9),0)
        #threshold
        ret,color_thresh = cv2.threshold(color_img, 127, 255, 0)

        # Need to find red on both ends of hsv scale
        if color == 'red':
            #define a range for color
            lower = self.colors[color+'2']['lower']
            upper = self.colors[color+'2']['upper']

            #threshold for only purple
            color_img = cv2.inRange(hsv,lower,upper)

            #blur the image
            color_img = cv2.GaussianBlur(color_img,(9,9),0)
            #threshold
            ret,color_thresh2 = cv2.threshold(color_img, 127, 255, 0)
            color_thresh = cv2.bitwise_or(color_thresh, color_thresh2)


        if self.showThreshold:
            cv2.imshow(color+' threshold',color_thresh)
            cv2.moveWindow(color+' threshold',300,0)

        exclude = []
        blocks = []

        while True:
            block = self.findBlock(color_thresh, exclude)
            if block == None:
                break
            h,w,points = block
            exclude.extend(points)
            if len(points) > 300.0/self.granularity:
                y,x = self.getCenter(points)
                blocks.append((y,x,points))

        self.blocks[color].extend(blocks)
        print color+' blocks found: '+str(len(blocks))

    def showImage(self):
        cv2.imshow('original',self.image)
        cv2.moveWindow('original',300,0)
        if self.debugColors:
            cv2.setMouseCallback("original", self.original_click)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def drawCenters(self,color):
        for block in self.blocks[color]:
            y,x,points = block
            self.drawPoint(self.image, (y,x))

    def findPoints(self,h,w,img,exclude):
        points = [(h,w)]
        i = 0
        while True:
            if i == len(points):
                break
            h,w = points[i]
            try:
                if img[h+self.granularity,w] > 0 and (h+self.granularity,w) not in points and (h+self.granularity,w) not in exclude:
                    points.append((h+self.granularity,w))
                if img[h-self.granularity,w] > 0 and (h-self.granularity,w) not in points and (h-self.granularity,w) not in exclude:
                    points.append((h-self.granularity,w))
                if img[h,w+self.granularity] > 0 and (h,w+self.granularity) not in points and (h,w+self.granularity) not in exclude:
                    points.append((h,w+self.granularity))
                if img[h,w-self.granularity] > 0 and (h,w-self.granularity) not in points and (h,w-self.granularity) not in exclude:
                    points.append((h,w-self.granularity))
                if img[h+self.granularity,w+self.granularity] > 0 and (h+self.granularity,w+self.granularity) not in points and (h+self.granularity,w+self.granularity) not in exclude:
                    points.append((h+self.granularity,w+self.granularity))
                if img[h-self.granularity,w-self.granularity] > 0 and (h-self.granularity,w-self.granularity) not in points and (h-self.granularity,w-self.granularity) not in exclude:
                    points.append((h-self.granularity,w-self.granularity))
                if img[h+self.granularity,w-self.granularity] > 0 and (h+self.granularity,w-self.granularity) not in points and (h+self.granularity,w-self.granularity) not in exclude:
                    points.append((h+self.granularity,w-self.granularity))
                if img[h-self.granularity,w+self.granularity] > 0 and (h-self.granularity,w+self.granularity) not in points and (h-self.granularity,w+self.granularity) not in exclude:
                    points.append((h-self.granularity,w+self.granularity))
            except:
                pass
            i+=1
        return points

    def findBlock(self,img, exclude):
        for h in range(0,self.height,self.granularity):
            for w in range(0,self.width,self.granularity):
                val = img[h,w]
                if val > 0 and (h,w) not in exclude:
                    return (h,w,self.findPoints(h,w,img,exclude))
        return None

    def getCenter(self,points):
        ha = 0
        wa = 0
        for h,w in points:
            ha += h
            wa += w

        ha = ha/len(points)
        wa = wa/len(points)
        return (ha,wa)

    def drawPoint(self,img,p):
        ha,wa = p
        img[ha,wa] = [0,0,255]
        img[ha+1,wa] = [0,0,255]
        img[ha-1,wa] = [0,0,255]
        img[ha,wa+1] = [0,0,255]
        img[ha,wa-1] = [0,0,255]
        img[ha+1,wa+1] = [0,0,255]
        img[ha-1,wa-1] = [0,0,255]
        img[ha-1,wa+1] = [0,0,255]
        img[ha+1,wa-1] = [0,0,255]

    def original_click(self,event, x, y, flags, param):
        #  self.drawPoint(self.image,(y,x))
        #  cv2.imshow('original',self.image)
         print("hsv:",x,y, self.image_hsv[y,x], event)

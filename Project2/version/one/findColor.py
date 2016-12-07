from blockVision import BlockVision
import cv2

if __name__ == '__main__':
  img = cv2.imread("./cam.png")
  cv2.imshow("img", img)
  cv2.waitKey()
  vision = BlockVision(img)
  vision.granularity = 3
  vision.showThreshold = True
  vision.debugColors = False
  vision.findBlocks("red")
  vision.drawAllCenters()
  for color, blocks in vision.blocks.iteritems():
      for y,x,p in blocks:
          print color+' block found at '+str(y)+', '+str(x)+' ('+str(len(p))+' points)'
  vision.showImage()

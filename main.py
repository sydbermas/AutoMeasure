import time, traceback
import cv2
from Camera import cameraThread
from Frame import frameObject
from Angles import distanceAngle
import numpy as np
import imutils
from imutils import perspective
from imutils import contours
from scipy.spatial import distance as dist


def run():

    try:

        # ------------------------------
        # set up cameras
        # ------------------------------

        # cameras variables
        left_camera_source = 0
        right_camera_source = 3
        pixel_width = 640
        pixel_height = 480
        angle_width = 78
        angle_height = 64  # 63
        frame_rate = 30
        camera_separation = 5 + 15 / 16

        # left camera 1
        ct1 = cameraThread.Camera_Thread()
        ct1.camera_source = left_camera_source
        ct1.camera_width = pixel_width
        ct1.camera_height = pixel_height
        ct1.camera_frame_rate = frame_rate

        # right camera 2
        ct2 = cameraThread.Camera_Thread()
        ct2.camera_source = right_camera_source
        ct2.camera_width = pixel_width
        ct2.camera_height = pixel_height
        ct2.camera_frame_rate = frame_rate

        # camera coding
        # ct1.camera_fourcc = cv2.VideoWriter_fourcc(*"YUYV")
        # ct2.camera_fourcc = cv2.VideoWriter_fourcc(*"YUYV")
        ct1.camera_fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        ct2.camera_fourcc = cv2.VideoWriter_fourcc(*"MJPG")

        # start cameras
        
        ct1.start()
        ct2.start()

        # ------------------------------
        # set up angles
        # ------------------------------

        # cameras are the same, so only 1 needed
        angler = distanceAngle.Frame_Angles(pixel_width, pixel_height, angle_width, angle_height)
        angler.build_frame()

        # ------------------------------
        # set up motion detection
        # ------------------------------

        # motion camera1
        # using default detect values
        targeter1 = frameObject.Frame_Object()
        targeter1.contour_min_area = 1
        targeter1.targets_max = 1
        targeter1.target_on_contour = True  # False = use box size
        targeter1.target_return_box = False  # (x,y,bx,by,bw,bh)
        targeter1.target_return_size = True  # (x,y,%frame)
        targeter1.contour_draw = True
        targeter1.contour_box_draw = True
        targeter1.targets_draw = True

        # motion camera2
        # using default detect values
        targeter2 = frameObject.Frame_Object()
        targeter2.contour_min_area = 1
        targeter2.targets_max = 1
        targeter2.target_on_contour = True  # False = use box size
        targeter2.target_return_box = False  # (x,y,bx,by,bw,bh)
        targeter2.target_return_size = True  # (x,y,%frame)
        targeter2.contour_draw = True
        targeter2.contour_box_draw = True
        targeter2.targets_draw = True

        # ------------------------------
        # stabilize
        # ------------------------------

        # pause to stabilize
        time.sleep(0.5)

        # ------------------------------
        # targeting loop
        # ------------------------------

        # variables
        maxsd = 2  # maximum size difference of targets, percent of frame
        klen = 3  # length of target queues, positive target frames required to reset set X,Y,Z,D

        # target queues
        x1k, y1k, x2k, y2k = [], [], [], []
        x1m, y1m, x2m, y2m = 0, 0, 0, 0

        # last positive target
        # from camera baseline midpoint
        X, Y, Z, D = 0, 0, 0, 0
        def rescale_frame(frame, percent=100):  # make the video windows a bit smaller     <<<<<<<<<<<<< UPDATE TO FRAMEOBJECT
            width = int(frame.shape[1] * percent/ 100)
            height = int(frame.shape[0] * percent/ 100)
            dim = (width, height)
            return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)


        def nothing(x): # for trackbar
            pass

        windowName="Webcam Live video feed"

        cv2.namedWindow(windowName,cv2.WINDOW_AUTOSIZE)

        
        
        # Sliders to adjust image
        # https://medium.com/@manivannan_data/set-trackbar-on-image-using-opencv-python-58c57fbee1ee
        cv2.createTrackbar("threshold",windowName, 80, 255, nothing)  
        cv2.createTrackbar("kernel", windowName, 5, 30, nothing)
        cv2.createTrackbar("iterations", windowName, 1, 10, nothing)

        showLive =True
        # loop
        while (showLive):

            # get frames
            frame1 = ct1.next(black=True, wait=1)
            frame2 = ct2.next(black=True, wait=1)

            # motion detection targets
            targets1 = targeter1.targets(frame1)
            targets2 = targeter2.targets(frame2)

            # check 1: motion in both frames
            if not (targets1 and targets2):
                x1k, y1k, x2k, y2k = [], [], [], []  # reset
            else:

                # split
                x1, y1, s1 = targets1[0]
                x2, y2, s2 = targets2[0]

                # check 2: similar size
                # if 100*(abs(s1-s2)/max(s1,s2)) > minsd:
                if abs(s1 - s2) > maxsd:
                    x1k, y1k, x2k, y2k = [], [], [], []  # reset
                else:

                    # update queues
                    x1k.append(x1)
                    y1k.append(y1)
                    x2k.append(x2)
                    y2k.append(y2)

                    # check 3: queues full
                    if len(x1k) >= klen:
                        # trim
                        x1k = x1k[-klen:]
                        y1k = y1k[-klen:]
                        x2k = x2k[-klen:]
                        y2k = y2k[-klen:]

                        # mean values
                        x1m = sum(x1k) / klen
                        y1m = sum(y1k) / klen
                        x2m = sum(x2k) / klen
                        y2m = sum(y2k) / klen

                        # get angles from camera centers
                        xlangle, ylangle = angler.angles_from_center(x1m, y1m, top_left=True, degrees=True)
                        xrangle, yrangle = angler.angles_from_center(x2m, y2m, top_left=True, degrees=True)

                        # triangulate
                        X, Y, Z, D = angler.location(camera_separation, (xlangle, ylangle), (xrangle, yrangle),
                                                     center=True, degrees=True)
           
            
            ret, frame = cameraThread.Camera_Thread.camera.read()
            frame_resize = rescale_frame(frame)
            if not ret:
                print("cannot capture the frame")
                exit()
   
            thresh= cv2.getTrackbarPos("threshold", windowName) 
            ret,thresh1 = cv2.threshold(frame_resize,thresh,255,cv2.THRESH_BINARY) 
    
            kern=cv2.getTrackbarPos("kernel", windowName) 
            kernel = np.ones((kern,kern),np.uint8) # square image kernel used for erosion
    
            itera=cv2.getTrackbarPos("iterations", windowName) 
            dilation = cv2.dilate(thresh1, kernel, iterations=itera)
            erosion = cv2.erode(dilation,kernel,iterations = itera) # refines all edges in the binary image

            opening = cv2.morphologyEx(erosion, cv2.MORPH_OPEN, kernel)
            closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)  
            closing = cv2.cvtColor(closing,cv2.COLOR_BGR2GRAY)          #<<<<<<<<<<<<<<<<<<<<<<< NO NEED
    
            contours,hierarchy = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE) # find contours with simple approximation cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE

            closing = cv2.cvtColor(closing,cv2.COLOR_GRAY2RGB)
            cv2.drawContours(closing, contours, -1, (128,255,0), 1)


             # focus on only the largest outline by area
            targets = [] #list to hold all areas

            for contour in contours:    #<<<<<<<<<<<<<<<<<<<<<<<WE NEED THIS!!!
                ar = cv2.contourArea(contour)
                targets.append(ar)

            max_area = max(targets)
            max_area_index = targets.index(max_area)  # index of the list element with largest area

            cnt = contours[max_area_index - 1] # largest area contour is usually the viewing window itself, why?

            cv2.drawContours(closing, [cnt], 0, (0,0,255), 1)
    
            def midpoint(ptA, ptB): 
                return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

            # compute the rotated bounding box of the contour
            orig = frame_resize.copy()
            box = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
            box = np.array(box, dtype="int")
    
            # order the points in the contour such that they appear
            # in top-left, top-right, bottom-right, and bottom-left
            # order, then draw the outline of the rotated bounding
            # box
            box = perspective.order_points(box)
            cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 1)
 
            # loop over the original points and draw them
            for (x, y) in box:
                cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

            # unpack the ordered bounding box, then compute the midpoint
            # between the top-left and top-right coordinates, followed by
            # the midpoint between bottom-left and bottom-right coordinates
            (tl, tr, br, bl) = box
            (tltrX, tltrY) = midpoint(tl, tr)
            (blbrX, blbrY) = midpoint(bl, br)
     
            # compute the midpoint between the top-left and top-right points,
            # followed by the midpoint between the top-righ and bottom-right
            (tlblX, tlblY) = midpoint(tl, bl)
            (trbrX, trbrY) = midpoint(tr, br)
     
            # draw the midpoints on the image
            cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
            cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
            cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
            cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
     
            # draw lines between the midpoints
            cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),(255, 0, 255), 1)
            cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),(255, 0, 255), 1)
            cv2.drawContours(orig, [cnt], 0, (0,0,255), 1)
    
            # compute the Euclidean distance between the midpoints
            dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
            dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))

            # compute the size of the object
            pixelsPerMetric = 1 # more to do here to get actual measurements that have meaning in the real world
            dimA = dA / pixelsPerMetric
            dimB = dB / pixelsPerMetric
 
            # draw the object sizes on the image
            cv2.putText(orig, "{:.1f}mm".format(dimA), (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(orig, "{:.1f}mm".format(dimB), (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # compute the center of the contour

            sd = frameObject.Frame_Object.safe_div
            M = cv2.moments(cnt)
            cX = int(sd(M["m10"],M["m00"]))
            cY = int(sd(M["m01"],M["m00"]))
 
            # draw the contour and center of the shape on the image
            cv2.circle(orig, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(orig, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 
            cv2.imshow(windowName, orig)
            cv2.imshow('', closing)
            if cv2.waitKey(30)>=0:
                showLive=False    


                                             

                # display frame
         
            # cv2.imshow("Left Camera 1", frame1)
            # cv2.imshow("Right Camera 2", frame2)

            # detect keys
            key = cv2.waitKey(1) & 0xFF
            if cv2.getWindowProperty(windowName, cv2.WND_PROP_VISIBLE) < 1:
                break
            # elif cv2.getWindowProperty('Right Camera 2', cv2.WND_PROP_VISIBLE) < 1:
            #     break
            elif key == ord('q'):
                break
            elif key != 255:
                print('KEY PRESS:', [chr(key)])

    # ------------------------------
    # full error catch
    # ------------------------------
    except:
        print(traceback.format_exc())

    # ------------------------------
    # close all
    # ------------------------------

    # close camera1
    try:
        ct1.stop()
    except:
        pass

    # close camera2
    try:
        ct2.stop()
    except:
        pass
    
   
    # kill frames
    cv2.destroyAllWindows()

    # done
    print('DONE')



if __name__ == '__main__':
    run()


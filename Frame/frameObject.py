import cv2
import numpy as np
import time

class Frame_Object:
    # ------------------------------
    # User Instructions
    # ------------------------------

    # # ------------------------------
    # # User Variables
    # # ------------------------------

    # # blur (must be positive and odd)
    # gaussian_blur = 15

    # # threshold
    # threshold = 15

    # # dilation
    # dilation_value = 6
    # dilation_iterations = 2
    # dilation_kernel = np.ones((dilation_value, dilation_value), np.uint8)

    # # contour size
    # contour_min_area = 10  # percent of frame area
    # contour_max_area = 80  # percent of frame area

    # # target select
    # targets_max = 4  # max targets returned
    # target_on_contour = True  # else use box size
    # target_return_box = False  # True = return (x,y,bx,by,bw,bh), else check target_return_size
    # target_return_size = False  # True = return (x,y,percent_frame_size), else just (x,y)

    # # display contour
    # contour_draw = True
    # contour_line = 1  # border width
    # contour_point = 4  # centroid point radius
    # contour_pline = -1  # centroid point line width
    # contour_color = (0, 255, 255)  # BGR color

    # # display contour box
    # contour_box_draw = True
    # contour_box_line = 1  # border width
    # contour_box_point = 4  # centroid point radius
    # contour_box_pline = -1  # centroid point line width
    # contour_box_color = (0, 255, 0)  # BGR color

    # # display targets
    # targets_draw = True
    # targets_point = 4  # centroid radius
    # targets_pline = -1  # border width
    # targets_color = (0, 0, 255)  # BGR color

    

    # # ------------------------------
    # # System Variables
    # # ------------------------------

    last_frame = None
    

    # ------------------------------
    # Functions
    # ------------------------------

    def targets(self, frame):

       

        # frame dimensions
        width, height, depth = np.shape(frame)
        area = width * height
       


    def safe_div(x,y): # so we don't crash so often 
        if y==0: return 0
        return x/y

    
    def loop(self):

        # load start frame
        frame = self.black_frame
        if not self.buffer.full():
            self.buffer.put(frame, False)

        # status
        self.frame_grab_on = True
        self.loop_start_time = time.time()

        # frame rate
        fc = 0
        t1 = time.time()

        # loop
        while 1:

            # external shut down
            if not self.frame_grab_run:
                break

            # true buffered mode (for files, no loss)
            if self.buffer_all:

                # buffer is full, pause and loop
                if self.buffer.full():
                    time.sleep(1 / self.camera_frame_rate)

                # or load buffer with next frame
                else:

                    grabbed, frame = self.camera.read()

                    if not grabbed:
                        break

                    self.buffer.put(frame, False)
                    self.frame_count += 1
                    fc += 1

            # false buffered mode (for camera, loss allowed)
            else:

                grabbed, frame = self.camera.read()
                if not grabbed:
                    break

                # open a spot in the buffer
                if self.buffer.full():
                    self.buffer.get()

                self.buffer.put(frame, False)
                self.frame_count += 1
                fc += 1

            # update frame read rate
            if fc >= 10:
                self.current_frame_rate = round(fc / (time.time() - t1), 2)
                fc = 0
                t1 = time.time()

        # shut down
        self.loop_start_time = 0
        self.frame_grab_on = False
        self.stop()
    
    
    

      


# ------------------------------
# Frame Angles and Distance
# ------------------------------

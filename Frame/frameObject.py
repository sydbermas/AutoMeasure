import cv2
import numpy as np

class Frame_Object:
    # ------------------------------
    # User Instructions
    # ------------------------------

    # ------------------------------
    # User Variables
    # ------------------------------

    # blur (must be positive and odd)
    gaussian_blur = 15

    # threshold
    threshold = 15

    # dilation
    dilation_value = 6
    dilation_iterations = 2
    dilation_kernel = np.ones((dilation_value, dilation_value), np.uint8)

    # contour size
    contour_min_area = 10  # percent of frame area
    contour_max_area = 80  # percent of frame area

    # target select
    targets_max = 4  # max targets returned
    target_on_contour = True  # else use box size
    target_return_box = False  # True = return (x,y,bx,by,bw,bh), else check target_return_size
    target_return_size = False  # True = return (x,y,percent_frame_size), else just (x,y)

    # display contour
    contour_draw = True
    contour_line = 1  # border width
    contour_point = 4  # centroid point radius
    contour_pline = -1  # centroid point line width
    contour_color = (0, 255, 255)  # BGR color

    # display contour box
    contour_box_draw = True
    contour_box_line = 1  # border width
    contour_box_point = 4  # centroid point radius
    contour_box_pline = -1  # centroid point line width
    contour_box_color = (0, 255, 0)  # BGR color

    # display targets
    targets_draw = True
    targets_point = 4  # centroid radius
    targets_pline = -1  # border width
    targets_color = (0, 0, 255)  # BGR color

    # ------------------------------
    # System Variables
    # ------------------------------

    last_frame = None

    # ------------------------------
    # Functions
    # ------------------------------

    def targets(self, frame):

        # frame dimensions
        width, height, depth = np.shape(frame)
        area = width * height

        # grayscale
        frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # blur
        frame2 = cv2.GaussianBlur(frame2, (self.gaussian_blur, self.gaussian_blur), 0)

        # initialize compare frame
        if self.last_frame is None:
            self.last_frame = frame2
            return []

        # delta
        frame3 = cv2.absdiff(self.last_frame, frame2)

        # threshold
        frame3 = cv2.threshold(frame3, self.threshold, 255, 0)[1]

        # dilation
        frame3 = cv2.dilate(frame3, self.dilation_kernel, iterations=self.dilation_iterations)
        

        # get contours
        contours, hierarchy = cv2.findContours(frame3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # targets
        targets = []
        for c in contours:

            # basic contour data
            ca = cv2.contourArea(c)
            bx, by, bw, bh = cv2.boundingRect(c)
            ba = bw * bh

            # target on contour
            if self.target_on_contour:
                p = 100 * ca / area
                if (p >= self.contour_min_area) and (p <= self.contour_max_area):
                    M = cv2.moments(c)  # ;print( M )
                    tx = int(M['m10'] / M['m00'])
                    ty = int(M['m01'] / M['m00'])
                    targets.append((p, tx, ty, bx, by, bw, bh, c))

            # target on contour box
            else:
                p = 100 * ba / area
                if (p >= self.contour_min_area) and (p <= self.contour_max_area):
                    tx = bx + int(bw / 2)
                    ty = by + int(bh / 2)
                    targets.append((p, tx, ty, bx, by, bw, bh, c))

        # select targets
        targets.sort()
        targets.reverse()
        targets = targets[:self.targets_max]

        # add contours to frame
        if self.contour_draw:
            for size, x, y, bx, by, bw, bh, c in targets:
                cv2.drawContours(frame, [c], 0, self.contour_color, self.contour_line)
                cv2.circle(frame, (x, y), self.contour_point, self.contour_color, self.contour_pline)

        # add contour boxes to frame
        if self.contour_box_draw:
            for size, x, y, bx, by, bw, bh, c in targets:
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), self.contour_box_color, self.contour_box_line)
                cv2.circle(frame, (bx + int(bw / 2), by + int(bh / 2)), self.contour_box_point, self.contour_box_color,
                           self.contour_box_pline)

        # add targets to frame
        if self.targets_draw:
            for size, x, y, bx, by, bw, bh, c in targets:
                cv2.circle(frame, (x, y), self.targets_point, self.targets_color, self.targets_pline)

        # # reset last frame
        # self.last_frame = frame2

        # return target x,y
        if self.target_return_box:
            return [(x, y, bx, by, bw, bh) for (size, x, y, bx, by, bw, bh, c) in targets]
        elif self.target_return_size:
            return [(x, y, size) for (size, x, y, bx, by, bw, bh, c) in targets]
        else:
            return [(x, y) for (size, x, y, bx, by, bw, bh, c) in targets]

    def frame_add_crosshairs(self, frame, x, y, r=20, lc=(0, 0, 255), cc=(0, 0, 255), lw=1, cw=1):

        x = int(round(x, 0))
        y = int(round(y, 0))
        r = int(round(r, 0))

        cv2.line(frame, (x, y - r * 2), (x, y + r * 2), lc, lw)
        cv2.line(frame, (x - r * 2, y), (x + r * 2, y), lc, lw)

        cv2.circle(frame, (x, y), r, cc, cw)
        


# ------------------------------
# Frame Angles and Distance
# ------------------------------

import cv2

cv2.namedWindow("AutoMeasure")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    vc.set(cv2.CAP_PROP_FRAME_WIDTH, 1366)
    vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    rval, frame = vc.read()

else:
    rval = False

while rval:
    cv2.imshow("AutoMeasure", frame)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    line_color = (0, 0, 255)
    thickness = 1
    type_ = cv2.LINE_AA
    pxstep = 16
    x = pxstep
    y = pxstep
    if key == 27: # exit on ESC
        break
    else:
        while x < frame.shape[1]:

            cv2.line(frame, (x, 16), (x, frame.shape[0]-16), color=line_color, lineType=type_, thickness=thickness)

            x += pxstep

            while y < frame.shape[0]:
                cv2.line(frame, (16, y), (frame.shape[1]-16, y), color=line_color, lineType=type_, thickness=thickness)

                y += pxstep
                break


vc.release()
cv2.destroyWindow("AutoMeasure")
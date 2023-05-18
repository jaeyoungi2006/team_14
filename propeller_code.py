# !/usr/bin/env python3
import rospy
import numpy as np
import cv2
import sys

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

import bisect

def Canny(img):
    color_plane = list(cv2.split(img))
    for index in range(3):
        color_plane[index] = cv2.Canny(color_plane[index], 50, 150)
    grey_Canny = cv2.Canny(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 50, 150)
    return color_plane[0] | color_plane[1] | color_plane[2] | grey_Canny


def get_cropped(img, img2):
    global lst

    contours, hierarchy = cv2.findContours(img2, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    contours = sorted(contours, key=lambda x: len(x))
    bestpoints = None
    largest_area = 0
    for c in contours[-10:]:
        c = c.reshape((-1, 2))
        x_y_sum = c[:, 0] + c[:, 1]
        x_y_dif = c[:, 0] - c[:, 1]

        f = [c[np.argmin(x_y_sum)], c[np.argmax(x_y_dif)], c[np.argmax(x_y_sum)], c[np.argmin(x_y_dif)]]


        points = np.round(np.array([f[0], f[1], f[2], f[3]], dtype="float32"))
        area = cv2.contourArea(points.reshape((1, -1, 2)))
        if area > largest_area:
            bestpoints = points
            largest_area = area

    p_flatten = bestpoints.reshape(-1)
    for x in range(8):
        if len(lst[x]) >= 150:
            lst[x] = lst[x][1:-1]
        lst[x].insert(bisect.bisect_left(lst[x], p_flatten[x]), p_flatten[x])
    bestpoints = np.array([np.mean(l[int(len(l) * 0.3) : int(len(l) * 0.7) + 1]) for l in lst], dtype="float32").reshape(-1, 2)


    width, height, _ = img.shape
    dst_points = np.array([[0, 0], [height - 1, 0], [height - 1, width - 1], [0, width - 1]], dtype="float32")

    M = cv2.getPerspectiveTransform(bestpoints, dst_points)

    return cv2.warpPerspective(img, M, (height, width))


def get_color(img):
    red, green, blue = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    pixel = img.shape[0] * img.shape[1]
    red = red.astype(float)
    blue = blue.astype(float)
    green = green.astype(float)
    b = np.sum((blue > red * 1.3) & (blue > green * 1.3))
    r = np.sum((red > blue * 1.3) & (red > green * 1.3))
    if b > pixel / 2:
        return '+1'
    if r > pixel / 2:
        return '-1'
    return '0'


class DetermineColor:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.color_pub = rospy.Publisher('/rotate_cmd', Header, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # cv2.imshow('Image', image)
            # cv2.waitKey(1)

            # prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # default: STOP

            # determine background color
            # TODO
            # determine the color and assing +1, 0, or, -1 for frame_id
            # msg.frame_id = '+1' # CCW (Blue background)
            # msg.frame_id = '0'  # STOP
            # msg.frame_id = '-1' # CW (Red background)

            img_canny = Canny(cv2.medianBlur(image, 3))
            img_cropped = get_cropped(image, img_canny)
            msg.frame_id = get_color(img_cropped)

            # cv2.imwrite("data/%06d.jpg"%self.count, image)

            # publish color_state
            # self.count += 1
            self.color_pub.publish(msg)

        except CvBridgeError as e:
            print(e)

    def rospy_shutdown(self, signal, frame):
        rospy.signal_shutdown("shut down")
        sys.exit(0)


if __name__ == '__main__':
    lst = [[] for _ in range(8)]
    detector = DetermineColor()
    rospy.init_node('CompressedImages1', anonymous=False)
    rospy.spin()
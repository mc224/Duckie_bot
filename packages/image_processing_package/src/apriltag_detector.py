import copy
import argparse
import cv2 as cv
from dt_apriltags import Detector


class AprilTagger:
    def __init__(self, detection_range):
        args = self.get_args()

        families = args.families
        nthreads = args.nthreads
        quad_decimate = args.quad_decimate
        quad_sigma = args.quad_sigma
        refine_edges = args.refine_edges
        decode_sharpening = args.decode_sharpening
        debug = args.debug

        self.at_detector = Detector(
            families=families,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=quad_sigma,
            refine_edges=refine_edges,
            decode_sharpening=decode_sharpening,
            debug=debug,
        )
        self.detection_range = detection_range

    @staticmethod
    def get_args():
        parser = argparse.ArgumentParser()

        parser.add_argument("--device", type=int, default=0)
        parser.add_argument("--width", help='cap width', type=int, default=960)
        parser.add_argument("--height", help='cap height', type=int, default=540)

        parser.add_argument("--families", type=str, default='tag36h11')
        parser.add_argument("--nthreads", type=int, default=1)
        parser.add_argument("--quad_decimate", type=float, default=2.0)
        parser.add_argument("--quad_sigma", type=float, default=0.0)
        parser.add_argument("--refine_edges", type=int, default=1)
        parser.add_argument("--decode_sharpening", type=float, default=0.25)
        parser.add_argument("--debug", type=int, default=0)

        args = parser.parse_args()

        return args

    def process_image(self, image):
        debug_image = copy.deepcopy(image)

        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(
            image,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )

        debug_image = self.draw_tags(debug_image, tags)

        return debug_image

    def draw_tags(self, image, tags):
        for tag in tags:
            tag_id = tag.tag_id
            center = tag.center
            corners = tag.corners

            center = (int(center[0]), int(center[1]))
            corner_01 = (int(corners[0][0]), int(corners[0][1]))
            corner_02 = (int(corners[1][0]), int(corners[1][1]))
            corner_03 = (int(corners[2][0]), int(corners[2][1]))
            corner_04 = (int(corners[3][0]), int(corners[3][1]))
            cv.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

            cv.line(image, (corner_01[0], corner_01[1]),
                    (corner_02[0], corner_02[1]), (255, 0, 0), 2)
            cv.line(image, (corner_02[0], corner_02[1]),
                    (corner_03[0], corner_03[1]), (255, 0, 0), 2)
            cv.line(image, (corner_03[0], corner_03[1]),
                    (corner_04[0], corner_04[1]), (0, 255, 0), 2)
            cv.line(image, (corner_04[0], corner_04[1]),
                    (corner_01[0], corner_01[1]), (0, 255, 0), 2)

            cv.putText(image, str(tag_id), (center[0] - 10, center[1] - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv.LINE_AA)

            if abs(int(corners[0][0]) - int(corners[1][0])) * abs(int(corners[1][1]) - int(corners[2][1])) > self.detection_range:
                f = open("packages/assets/signIds.txt", "w")
                f.write(str(tag_id))
                f.write("\n")
                f.close()

        return image
# test for detect_apriltag
# cum = cv.VideoCapture(0)
# april = AprilTagger()
# while True:
#     ret, image = cum.read()
#de
#     if not ret:
#         break
#     april.process_image(image)
#
#     key = cv.waitKey(1)
#     if key == 27:
#         break
# cv.destroyAllWindows()

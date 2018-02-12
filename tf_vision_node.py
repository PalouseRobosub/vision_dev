import argparse
import cv2
import numpy as np
import tensorflow as tf
import rospy
import cv_bridge

from rs_yolo.msg import Detection, DetectionArray
from sensor_msgs.msg import Image

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


class Node:
    def __init__(self, camera, label_map, graph):
        # Generate the session associated with the current graph.
        self.labels = label_map
        self.graph = graph
        self.session = tf.Session(graph=self.graph)
        self.sub = rospy.Subscriber('camera/{}/undistorted'.format(camera), Image, self.callback)
        self.pub = rospy.Publisher('vision/{}'.format(camera), DetectionArray, queue_size=10)
        self.pretty_pub = rospy.Publisher('pretty/vision/{}'.format(camera), Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
        self.min_score = 0.10

        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        self.d_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.d_scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.d_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.num_d = self.graph.get_tensor_by_name('num_detections:0')



    def process_image(self, image):

        # Run inference
        print 'Running inference'
        start_time = rospy.get_time()
        (boxes, scores, classes, num) = self.session.run(
                [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                feed_dict={
                    self.image_tensor: np.expand_dims(image, axis=0)
                })

        print 'Done in {} seconds'.format(rospy.get_time() - start_time)

        # all outputs are float32 numpy arrays, so convert types as appropriate
        output_dict = dict()
        output_dict['num_detections'] = int(num[0])
        output_dict['detection_classes'] = [int(x) for x in classes[0]]
        output_dict['detection_boxes'] = boxes[0]
        output_dict['detection_scores'] = scores[0]

        return output_dict


    def callback(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        with self.graph.as_default():
            detections = self.process_image(img_rgb)

        # Remove overlapping detections.
        kept_indices = tf.image.non_max_suppression(
               detections['detection_boxes'],
               detections['detection_scores'],
               10)

        with tf.Session() as sess:
            kept_indices_array = sess.run(kept_indices)

        detection_msg = DetectionArray()

        kept_indices = []
        for i in kept_indices_array.tolist():
            probability = detections['detection_scores'][i]
            if probability >= self.min_score:
                box = detections['detection_boxes'][i]
                classname = detections['detection_classes'][i]

                detection = Detection()
                detection.height = box[2] - box[0]
                detection.width = box[3] - box[1]
                detection.x = (box[3] + box[1]) / 2
                detection.y = (box[2] + box[0]) / 2
                detection.label = category_index[classname]['name']
                detection.probability = detections['detection_scores'][i]

                detection_msg.detections.append(detection)

                kept_indices.append(i)

        boxes = np.take(detections['detection_boxes'], kept_indices, axis=0)
        classes = np.take(detections['detection_classes'], kept_indices, axis=0)
        scores = np.take(detections['detection_scores'], kept_indices, axis=0)

        # Draw the detections for the pretty publisher.
        if len(kept_indices) > 0:
            vis_util.visualize_boxes_and_labels_on_image_array(
                    img,
                    boxes,
                    classes,
                    scores,
                    self.labels,
                    instance_masks=detections.get('detection_masks'),
                    use_normalized_coordinates=True,
                    min_score_thresh=self.min_score,
                    line_thickness=3)

        # Publish the detected images and the list of detections.
        self.pretty_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

        detection_msg.header.stamp = rospy.Time.now()
        self.pub.publish(detection_msg)


    def close(self):
        self.session.close()
        self.pub.unregister()
        self.sub.unregister()
        self.pretty_pub.unregister()


if __name__ == '__main__':
    rospy.init_node('tensorflow_detector')

    label_file = rospy.get_param('~labels')
    model_file = rospy.get_param('~model')
    camera = rospy.get_param('~camera')

    # Load the label map into TF to convert indices into labels.
    label_map = label_map_util.load_labelmap(label_file)
    categories = label_map_util.convert_label_map_to_categories(
            label_map, max_num_classes=90, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Load the model architecture into TF.
    detection_graph = tf.Graph()

    with detection_graph.as_default():
        graph_def = tf.GraphDef()

        with tf.gfile.GFile(model_file, 'rb') as f:
            serialized_graph = f.read()

        graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(graph_def, name='')

    # Construct the node and begin the pub/sub loops.
    node = Node(camera, category_index, detection_graph)

    rospy.spin()

    # Release the TF session.
    node.close()

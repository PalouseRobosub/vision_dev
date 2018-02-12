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


    def process_image(self, image):
        # Get handles to input and output tensors
        ops = self.graph.get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        tensor_dict = {}
        for key in [
            'num_detections', 'detection_boxes', 'detection_scores',
            'detection_classes', 'detection_masks'
        ]:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                tensor_dict[key] = self.graph.get_tensor_by_name(
                  tensor_name)

        if 'detection_masks' in tensor_dict:
            # The following processing is only for single image
            detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
            detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
            # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
            real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
            detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
            detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                detection_masks, detection_boxes, image.shape[0], image.shape[1])
            detection_masks_reframed = tf.cast(
                tf.greater(detection_masks_reframed, 0.5), tf.uint8)
            # Follow the convention by adding back the batch dimension
            tensor_dict['detection_masks'] = tf.expand_dims(
                detection_masks_reframed, 0)

        image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

        # Run inference
        print 'Running inference'
        start_time = rospy.get_time()
        output_dict = self.session.run(tensor_dict,
                               feed_dict={image_tensor: np.expand_dims(image, 0)})

        print 'Done in {} seconds'.format(rospy.get_time() - start_time)

        # all outputs are float32 numpy arrays, so convert types as appropriate
        output_dict['num_detections'] = int(output_dict['num_detections'][0])
        output_dict['detection_classes'] = output_dict[
            'detection_classes'][0].astype(np.uint8)
        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
        output_dict['detection_scores'] = output_dict['detection_scores'][0]

        if 'detection_masks' in output_dict:
            output_dict['detection_masks'] = output_dict['detection_masks'][0]

        return output_dict


    def callback(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        detections = self.process_image(img)

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

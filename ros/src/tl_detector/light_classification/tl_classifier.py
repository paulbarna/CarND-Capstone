from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import rospy
import yaml


class TLClassifier(object):
    def __init__(self):

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.safe_load(config_string)
        self.EnvSelection = self.config['is_site']

        root_path = r'light_classification/model'
        frozen_graph_name = 'frozen_inference_graph.pb'

        # give an indication of what model is used for either Sim or Site
        # RCNN model used for onsite validation due to better generalization
        # (however it is slow, it takes up to 3s to generate a detection)
        # SDD model used for sim validation becasue is much faster
        # (the traffic light changes quite often within the sim environment
        # compared to the real data)

        if self.EnvSelection:
            model_version = 'RCNN_Resnet101_Coco'
            rospy.loginfo("Site Env classifier trained with a COCO trained \
                           RCNN model : {}/{}".format(model_version,
                                                      frozen_graph_name))
        else:
            model_version = 'ssd_inception_v2_coco'
            rospy.loginfo("Sim Env classifier trained with a COCO trained \
                           SSD model : {}/{}".format(model_version,
                                                     frozen_graph_name))

        frozen_graph_path = '/'.join([root_path,
                                      model_version,
                                      frozen_graph_name])

        self.frozen_graph = tf.Graph()
        with self.frozen_graph.as_default():
            graph_defintion = tf.GraphDef()
            with tf.gfile.GFile(frozen_graph_path, 'rb') as fid:
                graph_defintion.ParseFromString(fid.read())
                tf.import_graph_def(graph_defintion, name='')

            self.graph_num_detections = self.frozen_graph.get_tensor_by_name(
                                            'num_detections:0')
            self.graph_image_tensor = self.frozen_graph.get_tensor_by_name(
                                            'image_tensor:0')
            self.graph_boxes = self.frozen_graph.get_tensor_by_name(
                                            'detection_boxes:0')
            self.graph_scores = self.frozen_graph.get_tensor_by_name(
                                            'detection_scores:0')
            self.graph_classes = self.frozen_graph.get_tensor_by_name(
                                            'detection_classes:0')

        self.sess = tf.Session(graph=self.frozen_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: traffic light color ID (specified in styx_msgs/TrafficLight)

        """
        with self.frozen_graph.as_default():

            img_np = np.expand_dims(image, axis=0)

            (graph_boxes,
             graph_scores,
             graph_classes,
             graph_num_detections) = self.sess.run([self.graph_boxes,
                                                    self.graph_scores,
                                                    self.graph_classes,
                                                    self.graph_num_detections],
                                         feed_dict={self.graph_image_tensor:
                                                        img_np})

        graph_boxes = np.squeeze(graph_boxes)
        graph_scores = np.squeeze(graph_scores)
        graph_classes = np.squeeze(graph_classes).astype(np.int32)

        # only return classifictions which have a
        # minimum probability distribution of 0.6
        if graph_scores[0] > .6:
            if graph_classes[0] == 1:
                print('green light')
                return TrafficLight.GREEN
            elif graph_classes[0] == 2:
                print('red light')
                return TrafficLight.RED
            elif graph_classes[0] == 3:
                print('yellow light')
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN

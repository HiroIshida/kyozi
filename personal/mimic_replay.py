#!/usr/bin/env python
import os
import rospkg
import rospy
from mimic.models import get_model_type_from_name
from mimic.scripts.predict import create_predictor_from_name

from kyozi.replay import Controller
from kyozi.utils import construct_config


if __name__=='__main__':
    rospy.init_node('visuo_motor_controller', anonymous=True)

    project_name = 'real_robot'
    model_name = 'LSTM'

    config_file = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config', 'pr2_rarm.yaml')
    config = construct_config(config_file)
    predictor = create_predictor_from_name(project_name, model_name)
    cont = Controller(predictor, config)
    cont.activate()
    rospy.spin()

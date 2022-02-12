#!/usr/bin/env python3
import rospy
from .exotica import ExoticaBase


class SingleArmExoticaEffFrame(ExoticaBase):

    def __init__(self):
        ExoticaBase.__init__(self)
        self.target_frame_id = rospy.get_param('~target_frame_id')
        self.base_frame_id = rospy.get_param('~base_frame_id')

    def reset(self, setup):
        # setup should be either length 3 (position), 6 (position + rotation[euler]), or 7 (position + rotation[quaternion])
        self.scene.attach_object_local(self.target_frame_id, self.base_frame_id, setup.data)
        self._did_recieve_setup = True

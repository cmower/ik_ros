#!/usr/bin/env python3
import rospy
from ik_ros.exotica import ExoticaBase
from ik_ros.ik import IKNode


class Exotica(ExoticaBase):

    def __init__(self):
        ExoticaBase.__init__(self)
        self.target_frame_id = rospy.get_param('~target_frame_id')
        self.base_frame_id = rospy.get_param('~base_frame_id')
        self._did_recieve_setup = False

    def reset(self, setup):
        # setup should be either length 3 (position), 6 (position + rotation[euler]), or 7 (position + rotation[quaternion])
        self.scene.attach_object_local(self.target_frame_id, self.base_frame_id, setup.data)
        self._did_recieve_setup = True

    def did_recieve_setup(self):
        return self._did_recieve_setup


def main():
    IKNode(Exotica).spin()


if __name__ == '__main__':
    main()

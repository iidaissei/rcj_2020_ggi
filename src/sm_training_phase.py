#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: RCJ2020_GoGetItのTrainingPhase用プログラム
# Author: Issei Iida
# Date: 2020/02/16
#---------------------------------------------------------------------

# Python
import sys
# ROS
import rospy
import rosparam
import smach
import smach_ros
from std_msgs.msg import String
from rcj_2020_ggi.srv import GgiTestPhase
from mimi_common_pkg.srv import LocationSetup
from ggi.srv import ListenCommand, GgiLearning, YesNo

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import BaseCarrier, speak
from common_action_client import enterTheRoomAC


class Listen(smach.State): 
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['listen_failed',
                            'motion',
                            'event'],
                output_keys = ['cmd_output'])
        # Service
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)
        # Value
        self.cmd_state = rosparam.get_param('/ggi/cmd_state')

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN')
        result = self.listen_cmd_srv(file_name = 'voice_cmd')
        if result.result:
            command = result.cmd
            userdata.cmd_output = command
            state = self.cmd_state[command]
            return state
        else:
            speak("I could't hear")
            return 'listen_failed'


class Motion(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_motion'],
                input_keys = ['cmd_input'])
        self.bc = BaseCarrier()
        # Publisher
        self.pub_follow_req = rospy.Publisher('/chase/request', String, queue_size = 1)
 
    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        if userdata.cmd_input == 'turn right':
            self.bc.angleRotation(-45)
        elif userdata.cmd_input == 'turn left':
            self.bc.angleRotation(45)
        elif userdata.cmd_input == 'go straight':
            self.bc.translateDist(0.20)
        elif userdata.cmd_input == 'go back':
            self.bc.translateDist(-0.20)
        elif userdata.cmd_input == 'right face':
            self.bc.angleRotation(-90)
        elif userdata.cmd_input == 'left face':
            self.bc.angleRotation(90)
        elif userdata.cmd_input == 'about face':
            self.bc.angleRotation(-180)
        elif userdata.cmd_input == 'follow me':
            speak('Start follow')
            self.pub_follow_req.publish('start')
        elif userdata.cmd_input == 'stop following':
            speak('Stop follow')
            self.pub_follow_req.publish('stop')
        else:
            pass
        return 'finish_motion'


class Event(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_event', 'finish_test_phase'],
                input_keys = ['cmd_input'])
        # Survice
        self.ggi_learning_srv = rospy.ServiceProxy('/ggi_learning', GgiLearning)
        self.location_setup_srv = rospy.ServiceProxy('/location_setup', LocationSetup)
        self.test_phase_srv = rospy.ServiceProxy('/ggi/testphase', GgiTestPhase)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)

    def execute(self, userdata):
        rospy.loginfo('Executing state: EVENT')
        if userdata.cmd_input == 'start go get it':
            speak('Start go get it')
            enterTheRoomAC(0.8)
        elif userdata.cmd_input == 'start ggi learning':
            result = self.ggi_learning_srv().location_name
            print result
            self.location_setup_srv(state = 'add', name = result)
            speak('Location added')
        elif userdata.cmd_input == 'save location':
            speak('Can i save?')
            result = self.yesno_srv()
            if result.result:
                self.location_setup_srv(state = 'add', name = 'operator')
                self.location_setup_srv(state = 'save', name = 'ggi')
                speak('Location saved')
            else:
                speak('OK')
        elif userdata.cmd_input == 'start test phase':
            speak('Can i start?')
            result = self.yesno_srv()
            if result.result:
                speak('Start the test phase')
                result = self.test_phase_srv().result
                speak('Finish test phase')
                return 'finish_test_phase'
            else:
                speak('OK')
        else:
            pass
        return 'finish_event'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])

    with sm_top:
        smach.StateMachine.add(
                'LISTEN',
                Listen(),
                transitions = {'listen_failed':'LISTEN',
                               'motion':'MOTION',
                               'event':'EVENT'},
                remapping = {'cmd_output':'cmd_name'})

        smach.StateMachine.add(
                'MOTION',
                Motion(),
                transitions = {'finish_motion':'LISTEN'},
                remapping = {'cmd_input':'cmd_name'})

        smach.StateMachine.add(
                'EVENT',
                Event(),
                transitions = {'finish_event':'LISTEN',
                               'finish_test_phase':'finish_sm_top'},
                remapping = {'cmd_input':'cmd_name'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_training_phsase', anonymous = True)
    main()

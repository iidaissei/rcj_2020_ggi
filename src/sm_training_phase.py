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
from ggi.srv import ListenCommand, GgiLearning, YesNo
from rcj_2020_ggi.srv import GgiTestPhase
from mimi_common_pkg.srv import LocationSetup

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import BaseCarrier, speak


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
        self.cmd_dict = rosparam.get_param('/ggi/voice_cmd')

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN')
        result = self.listen_cmd_srv(file_name = 'voice_cmd')
        if result.result:
            command = result.cmd
            userdata.cmd_output = command
            userdata.words_output = self.words_dict[command]
            state = self.cmd_dict[command]
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

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        if userdata.cmd_input == 'turn right':
            speak('Rotate right')
            self.bc.angleRotation(-45)
        elif userdata.cmd_input == 'turn left':
            speak('Rotate left')
            self.bc.angleRotation(45)
        elif userdata.cmd_input == 'go straight':
            speak('Go forward')
            self.bc.translateDist(0.20)
        elif userdata.cmd_input == 'go back':
            speak('Go back')
            self.bc.translateDist(-0.20)
        else:
            pass
        return 'finish_motion'


class Event(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_event', 'finish_test_phase'],
                input_keys = ['cmd_input'])
        # Publisher
        self.pub_follow_req = rospy.Publisher('/chase/request', String, queue_size = 1)
        # Survice
        self.ggi_learning_srv = rospy.ServiceProxy('/ggi_learning', GgiLearning)
        self.location_setup_srv = rospy.ServiceProxy('/location_setup', LocationSetup)
        self.test_phase_srv = rospy.ServiceProxy('/ggi_test_phase', GgiTestPhase)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)

    def execute(self, userdata):
        rospy.loginfo('Executing state: EVENT')
        if userdata.cmd_input == 'follow me':
            speak('I will follow you')
            self.pub_follow_req.publish('start')
        elif userdata.cmd_input == 'stop following':
            speak('Stop following')
            self.pub_follow_req.publish('stop')
        elif userdata.cmd_input == 'start ggi training':
            speak('Start ggi learning')
            result = self.ggi_learning_srv().location_name
            print result
            self.location_setup_srv(state = 'add', name = result.location_name)
            speak('Location added')
        elif userdata.cmd_input == 'save location':
            self.file_name = self.listen_cmd_srv(file_name = 'map_name').cmd
            print self.file_name
            speak('Name is ', self.file_name)
            speak('Is this ok?')
            result = self.yesno_srv()
            if result.result:
                self.location_setup_srv(state = 'save', name = self.file_name)
                speak('Location saved')
            else:
                speak('Say the command again')

        elif userdata.cmd_input == 'start test phase':
            speak('Can i start the test phase?')
            result = self.yesno_srv()
            if result.result:
                speak('Start the test phase')
                result = self.test_phase_srv().result
                print result
                speak('Finish test phase')
                return 'finish_test_phase'
            else:
                speak('OK, Continue training')
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

#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: RCJ2020_GoGetItのTestPhase用プログラム
# Author: Issei Iida
# Date: 2020/02/12
# Memo: オーダーはBringmeなので、現在位置確認処理は省く
#----------------------------------------------------------

# Python
import sys
# ROS
import rospy
import smach
import smach_ros
from rcj_2020_ggi.srv import GgiTestPhase, GgiTestPhaseResponse

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import *
from common_function import *


class DecideMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['decide_finish'])
        # Subscriber
        self.posi_sub = rospy.Subscriber('/navigation/move_place', String, self.currentPosiCB)
        # Value
        self.operator_coord = searchLocationName('operator')
        self.current_position = 'operator'

    def currentPosiCB(self, data):
        self.current_position = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: DECIDE_MOVE')
        if self.current_position != 'operator':
            navigationAC(self.operator_coord)
            speak('I arrived operator')
        else:
            pass
        return 'decide_finish'


class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failure',
                                         'next_cmd',
                                         'all_cmd_finish'],
                             output_keys = ['cmd_out'])
        # ServiceProxy
        # self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        # Value
        self.listen_count = 1
        self.cmd_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_COMMAND')
        if self.cmd_count == 4:
            speak('Finish all command')
            return 'all_cmd_finish'
        elif self.listen_count <= 3:
            speak('CommandNumber is ' + str(self.cmd_count))
            speak('ListenCount is ' + str(self.listen_count))
            speak('Please instruct me')
            # result = self.listen_srv()
            result = True
            if result:
            # if result.result:
                self.listen_count = 1
                self.cmd_count += 1
                userdata.cmd_out = result
                return 'listen_success'
            else:
                self.listen_count += 1
                speak("Sorry, I could't listen")
                return 'listen_failure'
        else:
            speak("I couldn't understand the instruction")
            self.listen_count = 1
            self.cmd_count +=1
            return 'next_cmd'


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failure'],
                             input_keys = ['cmd_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXE_ACTION')
        # action = userdata.cmd_in.action
        # data = userdata.cmd_in.data
        action = ['go']
        data = ['table']
        result = exeActionPlanAC(action, data)
        if result:
            speak('Action success')
            return 'action_success'
        else:
            speak('Action failed')
            return 'action_failure'


class GgiTestPhaseServer():
    def __init__(self):
        self.server = rospy.Service('/ggi/testphase', GgiTestPhase, self.startSM)

    def startSM(self, req):
        sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
        with sm_top:
            smach.StateMachine.add(
                    'DECIDE_MOVE',
                    DecideMove(),
                    transitions = {'decide_finish':'LISTEN_COMMAND'})

            smach.StateMachine.add(
                    'LISTEN_COMMAND',
                    ListenCommand(),
                    transitions = {'listen_success':'EXE_ACTION',
                                   'listen_failure':'LISTEN_COMMAND',
                                   'next_cmd':'DECIDE_MOVE',
                                   'all_cmd_finish':'finish_sm_top'},
                    remapping = {'cmd_out':'cmd'})

            smach.StateMachine.add(
                    'EXE_ACTION',
                    ExeAction(),
                    transitions = {'action_success':'DECIDE_MOVE',
                                   'action_failure':'DECIDE_MOVE'},
                    remapping = {'cmd_in':'cmd'})

        outcomes = sm_top.execute()

        return GgiTestPhaseResponse(result = True)


if __name__ == '__main__':
    rospy.init_node('sm_test_phase', anonymous = True)
    m = GgiTestPhaseServer()
    rospy.spin()

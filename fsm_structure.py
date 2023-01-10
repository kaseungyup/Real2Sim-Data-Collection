#!/usr/bin/env python

import rospy
import smach

traj_num = 5

class Spc(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['gotoLPC','complete'],
                             input_keys=['spc_counter_in'],
                             output_keys=['spc_counter_out'])
        self.traj_num = traj_num 

    def execute(self, userdata):
        # rospy.loginfo('Executing state SPC')
        if userdata.spc_counter_in < self.traj_num+1:
            userdata.spc_counter_out = userdata.spc_counter_in + 1
            rospy.loginfo("Epoch number : %d"%userdata.spc_counter_in)
            return 'gotoLPC'
        else:
            rospy.loginfo("Complete")
            return 'complete'

class Lpc(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['gotoSPC','sendtraj'],
                             input_keys=['lpc_counter_in'])
        self.traj_num = traj_num
        
    def execute(self, userdata):
        # rospy.loginfo('Executing state LPC')
        if userdata.lpc_counter_in < self.traj_num+1:
            rospy.loginfo('Saving trajectory number %d'%userdata.lpc_counter_in)        
            return 'gotoSPC'
        else:
            rospy.loginfo("Sending trajectory")
            return 'sendtraj'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SPC', Spc(), 
                               transitions={'gotoLPC':'LPC', 
                                            'complete':'exit'},
                               remapping={'spc_counter_in':'sm_counter', 
                                          'spc_counter_out':'sm_counter'})
        smach.StateMachine.add('LPC', Lpc(), 
                               transitions={'gotoSPC':'SPC',
                                            'sendtraj':'SPC'},
                               remapping={'lpc_counter_in':'sm_counter'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
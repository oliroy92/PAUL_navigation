#!/usr/bin/env python

import rospy
import smach
import smach_ros

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done', 'e_stop'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state INIT')
        if userdata.e_stop:
            return 'e_stop'
        return 'init_done'

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'start_mapping', 'order_received', 'e_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE')
        if userdata.e_stop:
            return 'e_stop'
        elif userdata.mapping:
            return 'start_mapping'
        elif userdata.order_received:
            return 'order_received'
        return 'wait'
        
class Mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done_mapping', 'e_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MAPPING')
        if userdata.e_stop:
            return 'e_stop'
        return 'done_mapping'

# processing state machine
class Processing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cancel_order', 'done_order', 'e_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PROCESSING')
        if userdata.e_stop:
            return 'e_stop'
        elif userdata.cancel_order:
            return 'cancel_order'
        return 'done_order'

class PlanTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trajectory', 'e_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PLAN TRAJECTORY')
        if userdata.e_stop:
            return 'e_stop'
        return 'trajectory'

class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done_moving', 'e_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVING')
        # for waypoint in waypoints:
        if userdata.e_stop:
            return 'e_stop'
        return 'done_moving'

class PickItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['item_picked', 'e_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PICK ITEM')
        if userdata.e_stop:
            return 'e_stop'
        return 'item_picked'


# main state machine
class Unloading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done_unloading', 'e_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UNLOADING')
        if userdata.e_stop:
            return 'e_stop'
        return 'done_unloading'

class EmergencyStopped(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reset'])

    def execute(self, userdata):
        rospy.loginfo('Executing state EMERGENCY STOPPED')
        return 'reset'

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])
    sm.userdata.e_stop = 0
    sm.userdata.mapping = 0
    sm.userdata.order_received = 0
    sm.userdata.cancel_order = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', Init(), 
                               transitions={'init_done':'IDLE', 
                                            'e_stop':'EMERGENCY STOPPED'})
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'wait':'IDLE',
                                            'start_mapping':'MAPPING',
                                            'order_received':'PROCESSING',
                                            'e_stop':'EMERGENCY STOPPED'})
        smach.StateMachine.add('MAPPING', Mapping(), 
                               transitions={'done_mapping':'IDLE',
                                            'e_stop':'EMERGENCY STOPPED'})

        # Create the sub SMACH state machine
        processing_sm = smach.StateMachine(outcomes=['cancel_order', 'done_order', 'e_stop'])

        # Open the container
        with processing_sm:

            # Add states to the container
            smach.StateMachine.add('PLAN TRAJECTORY', PlanTrajectory(), 
                                   transitions={'trajectory':'MOVING', 
                                                'e_stop':'e_stop'})
            smach.StateMachine.add('MOVING', Moving(), 
                                   transitions={'done_moving':'PICK ITEM', 
                                                'e_stop':'e_stop'})
            smach.StateMachine.add('PICK ITEM', PickItem(), 
                                   transitions={'item_picked':'done_order', 
                                                'e_stop':'e_stop'})

        smach.StateMachine.add('PROCESSING', processing_sm, 
                               transitions={'cancel_order':'IDLE',
                                            'done_order':'UNLOADING',
                                            'e_stop':'EMERGENCY STOPPED'})

        smach.StateMachine.add('UNLOADING', Unloading(), 
                               transitions={'done_unloading':'IDLE',
                                            'e_stop':'EMERGENCY STOPPED'})
        smach.StateMachine.add('EMERGENCY STOPPED', EmergencyStopped(), 
                               transitions={'reset':'INIT'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    # outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
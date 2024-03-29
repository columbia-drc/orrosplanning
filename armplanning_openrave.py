#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2010 Rosen Diankov (rosen.diankov@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'
import roslib; roslib.load_manifest('orrosplanning')
import rospy

from optparse import OptionParser
from openravepy import *
from openravepy.misc import OpenRAVEGlobalArguments
from numpy import *
import numpy,time,threading
from itertools import izip
import tf
import os
import orrosplanning.srv
import sensor_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import ipdb
import time
import copy
from cartesian_trajectory_retiming import *
from trajectory_server import *

if __name__ == "__main__":
    parser = OptionParser(description='openrave planning example')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='robots/pr2-beta-static.zae',
                      help='scene to load (default=%default)')
    parser.add_option('--collision_map',action="store",type='string',dest='collision_map',default='/collision_map/collision_map',
                      help='The collision map topic (maping_msgs/CollisionMap), by (default=%default)')
    parser.add_option('--ipython', '-i',action="store_true",dest='ipython',default=False,
                      help='if true will drop into the ipython interpreter rather than spin')
    parser.add_option('--mapframe',action="store",type='string',dest='mapframe',default=None,
                      help='The frame of the map used to position the robot. If --mapframe="" is specified, then nothing will be transformed with tf')
    parser.add_option('--jitter',action="store",type='float',dest='jitter',default=None,
                      help='The jitter to use when moving robot out of collision')
    parser.add_option('--maxvelmult',action='store',type='float',dest='maxvelmult',default=1.0,
                      help='The maximum velocity multiplier when timing the trajectories (default=%default)')
    parser.add_option('--wait-for-collisionmap',action='store',type='float',dest='wait_for_collisionmap',default=None,
                      help='Time (seconds) to set. Will wait for the collision map time stamp to reach the same time as the service being called. If 0, will wait indefinitely.')
    parser.add_option('--request-for-joint_states',action='store',type='string',dest='request_for_joint_states',default='topic',
                      help='whether to get joint states from topic or service. If ="service", will not update robot joint states until receiving service call.')
    parser.add_option('--use-simulation',action='store',type='string',dest='simulation',default=None,
                      help='if use-simulation is set, we dismiss timestamp of collisionmap.')
    (options, args) = parser.parse_args()
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=False)
    RaveLoadPlugin(os.path.join(roslib.packages.get_pkg_dir('orrosplanning'),'lib','liborrosplanning.so'))
    #RaveLoadPlugin(os.path.join(roslib.packages.get_pkg_dir('openraveros'),'lib','openraveros'))
    namespace = 'openrave'
    #env.AddModule(RaveCreateModule(env,'rosserver'),namespace)
    rospy.loginfo('initializing, please wait for ready signal...')
    handles = [] # for viewer

    joint_sender = JointAngleSender(4765)
    


    def PlayTrajectory(env, robot, traj, goal, step_size = .1):
  #      ipdb.set_trace()
        traj.Insert(0, robot.GetActiveDOFValues(), robot.GetActiveConfigurationSpecification(), False)
        traj_retimed = simple_retime(env, robot, traj, goal)
        summary_joint_list = []
        for i in xrange(traj.GetNumWaypoints()):
            joints = traj.GetConfigurationSpecification().ExtractJointValues(traj.GetWaypoint(i),robot, robot.GetActiveDOFIndices())
            summary_joint_list.append(joints)
            
        joint_list = []
        inp = 'p'
        while inp == 'p':
            joint_list = []
            for i in xrange(traj_retimed.GetNumWaypoints()):
                
                joints = traj_retimed.GetConfigurationSpecification().ExtractJointValues(traj_retimed.GetWaypoint(i),robot, robot.GetActiveDOFIndices())
                joint_list.append(joints.tolist())
                robot.SetActiveDOFValues(joints)
                env.UpdatePublishedBodies()
                time.sleep(step_size)
            inp = raw_input('Send trajectory to robot (y)es/(n)o/(p)lay again')
        if inp == 'y':
            joint_sender.send_trajectory(summary_joint_list,robot.GetActiveManipulator().GetName())

    def PlanFromPose(msg,side):
        req = orrosplanning.srv.MoveToHandPositionRequest()
        
        req.hand_frame_id = "Body_%sWP"%(side[0].upper())
        req.manip_name = "%sArm"%(side)
        req.hand_goal = msg
        resp = MoveToHandPositionFn(req)
        
    
    def UpdateRobotJoints(msg):
        global options
        with envlock:
            with env:
                for name,pos in izip(msg.name,msg.position):
                    j = robot.GetJoint(name)
                    if j is not None:
                        values[j.GetDOFIndex()] = pos
                    robot.SetDOFValues(values)
    def UpdateRobotJointsFn(req):
        rospy.loginfo("Update joint states")
        UpdateRobotJoints(req.jointstate)
        return True
    try:
        rospy.init_node('armplanning_openrave',disable_signals=False)
        with env:
            env.Load(options.scene)
            robot = env.GetRobots()[0]

            #set robot weights/resolutions (without this planning will be slow)
            if 1:
                lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
                if not lmodel.load():
                    lmodel.autogenerate()
                lmodel.setRobotWeights()
                lmodel.setRobotResolutions()
                resolutions=robot.GetDOFResolutions()
                weights=robot.GetDOFWeights()
                for j in robot.GetJoints():
                    j.SetResolution(3.0*resolutions[j.GetDOFIndex():(j.GetDOFIndex()+j.GetDOF())])
                    j.SetWeights(1.0*weights[j.GetDOFIndex():(j.GetDOFIndex()+j.GetDOF())])
            # create ground right under the robot
            ab=robot.ComputeAABB()
            ground=RaveCreateKinBody(env,'')
            ground.SetName('map')
            ground.InitFromBoxes(array([r_[ab.pos()-array([0,0,ab.extents()[2]+0.002]),2.0,2.0,0.001]]),True)
            env.AddKinBody(ground,False)
            if options.mapframe is None:
                options.mapframe = robot.GetLinks()[0].GetName()
                print 'setting map frame to %s'%options.mapframe
            collisionmap = RaveCreateSensorSystem(env,'CollisionMap expirationtime 10000 bodyoffset %s topic %s prunecollisions 1 .01'%(robot.GetName(),options.collision_map))
            if options.request_for_joint_states == 'topic':
                sub = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, UpdateRobotJoints,queue_size=1)
                # joint_states = '/joint_states'
                # for a in args:
                #     if a.startswith('%s:='%joint_states):
                #         joint_states = a[len(joint_states)+2:]
                # robot.SetController(RaveCreateController(env,'ROSPassiveController jointstate %s'%joint_states),range(robot.GetDOF()),False)

        # have to do this manually because running linkstatistics when viewer is enabled segfaults things
        if options._viewer is None:
            env.SetViewer('qtcoin')
        elif len(options._viewer) > 0:
            env.SetViewer(options._viewer)
        if len(options.mapframe) > 0:
            listener = tf.TransformListener()
        values = robot.GetDOFValues()
        envlock = threading.Lock()

        postprocessing = ['shortcut_linear','<_nmaxiterations>20</_nmaxiterations><_postprocessing planner=\"parabolicsmoother\"><_nmaxiterations>40</_nmaxiterations></_postprocessing>']
        def MoveToHandPositionFn(req, lift_movement = [0,0,-.1]):
            global options
            rospy.loginfo("MoveToHandPosition")
            
            try:
                with envlock:
                    if options.wait_for_collisionmap is not None:
                        starttime=time.time()
                        handgoalstamp = int64(req.hand_goal.header.stamp.to_nsec())
                        while True:
                            timepassed = time.time()-starttime
                            collisionstamp = collisionmap.SendCommand("gettimestamp")
                            if collisionstamp is not None:
                                if int64(collisionstamp)-handgoalstamp >= 0:
                                     break
                            if options.wait_for_collisionmap > 0 and timepassed > options.wait_for_collisionmap:
                                if options.simulation is not None:
                                    break;
                                raise ValueError('failed to acquire new collision map, collision timestamp is %s, service timestamp is %s'%(collisionstamp,handgoalstamp))
                            time.sleep(0.1) # wait

                    collisionmap.SendCommand("collisionstream 0")

                    with env:
                        basemanip = interfaces.BaseManipulation(robot,plannername=None if len(req.planner)==0 else req.planner,maxvelmult=options.maxvelmult)
                        rospy.loginfo("MoveToHandPosition2")
                        if len(options.mapframe) > 0:
                            (robot_trans,robot_rot) = listener.lookupTransform(options.mapframe, robot.GetLinks()[0].GetName(), rospy.Time(0))
                            Trobot = matrixFromQuat([robot_rot[3],robot_rot[0],robot_rot[1],robot_rot[2]])
                            Trobot[0:3,3] = robot_trans
                            robot.SetTransform(Trobot)
                            hand = listener.transformPose(options.mapframe, req.hand_goal)
                        else:
                            hand = req.hand_goal
                        o = hand.pose.orientation
                        p = hand.pose.position
                        Thandgoal = matrixFromQuat([o.w,o.x,o.y,o.z])
                        Thandgoal[0:3,3] = [p.x,p.y,p.z]

                        if len(req.manip_name) > 0:
                            manip = robot.GetManipulator(req.manip_name)
                            if manip is None:
                                rospy.logerr('failed to find manipulator %s'%req.manip_name)
                                return None
                        else:
                            manips = [manip for manip in robot.GetManipulators() if manip.GetEndEffector().GetName()==req.hand_frame_id]
                            if len(manips) == 0:
                                rospy.logerr('failed to find manipulator end effector %s'%req.hand_frame_id)
                                return None
                            manip = manips[0]
                            
                        robot.SetActiveManipulator(manip)
                        robot.SetActiveDOFs(manip.GetArmIndices())
                        if len(req.hand_frame_id) > 0:
                            handlink = robot.GetLink(req.hand_frame_id)
                            if handlink is None:
                                rospy.logerr('failed to find link %s'%req.hand_frame_id)
                                return None
                            Thandlink = handlink.GetTransform()
                        else:
                            Thandlink = manip.GetEndEffectorTransform()
                        if manip.GetIkSolver() is None:
                            rospy.loginfo('generating ik for %s'%str(manip))
                            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
                            if not ikmodel.load():
                                ikmodel.autogenerate()

                        
                        Tgoalee = dot(Thandgoal,dot(linalg.inv(Thandlink),manip.GetTransform()))
                        
                        try:
                            starttime = time.time()
                            joint_goals = manip.FindIKSolutions(Tgoalee,IkFilterOptions.CheckEnvCollisions)
                            ipdb.set_trace()
                            current_pose = robot.GetActiveDOFValues()
                            indices = sum((joint_goals - current_pose)**2, axis=-1).argsort()
                            
                            sorted_joint_goals = joint_goals[indices]
                            num_goals = 5
                            traj_dict = {}

                            if lift_movement:
                                motion = numpy.array(lift_movement)
                                direction = motion/numpy.linalg.norm(motion)
                                stepsize = .003
                                minsteps = maxsteps = numpy.linalg.norm(motion) / stepsize
                                reachable_joint_goals = []
                                with robot:
                                    for joint_goal in joint_goals:
                                        try:
                                            robot.SetActiveDOFValues(joint_goal)
                                            trajup = basemanip.MoveHandStraight(direction=direction, stepsize=stepsize,minsteps=minsteps,maxsteps=maxsteps,execute=False,outputtrajobj=True)
                                            reachable_joint_goals += [joint_goal]
                                            traj_dict[' '.join([str(num) for num in joint_goal])] = trajup
                                            if len(reachable_joint_goals) >= num_goals:
                                                break
                                            
                                        except:
                                            print "move straight failed"
                                sorted_joint_goals = numpy.array(reachable_joint_goals)

                            

                            
                            #traj = basemanip.MoveToHandPosition(matrices=[Tgoalee],maxtries=3,seedik=120,execute=False,outputtrajobj=True,maxiter=750,jitter=options.jitter,postprocessing=postprocessing)

                            traj = basemanip.MoveManipulator(goals=sorted_joint_goals[:5],execute=False,outputtrajobj=True,maxiter=750,jitter=options.jitter)
                            if lift_movement:
                                try:
                                    joint_id = ' '.join([str(num) for num in traj.GetWaypoint(traj.GetNumWaypoints() - 1)][:7])
                                    ipdb.set_trace()
                                    found_traj = traj_dict[joint_id]
                                    for traj_num in xrange(found_traj.GetNumWaypoints()):
                                        datapoint = numpy.concatenate((found_traj.GetWaypoint(traj_num),numpy.array([0])))
                                        
                                        traj.Insert(traj.GetNumWaypoints(), datapoint)
                                except:
                                    ipdb.set_trace()
                            rospy.loginfo('total planning time: %fs'%(time.time()-starttime))
                        except:
                            rospy.logerr('failed to solve for T=%s, error messages are:'%repr(Tgoalee))
                            RaveSetDebugLevel(DebugLevel.Verbose)
                            manip.FindIKSolution(Tgoalee,IkFilterOptions.CheckEnvCollisions)
                            RaveSetDebugLevel(DebugLevel.Debug)
                            return None
                        # parse trajectory data into the ROS structure
                        res = orrosplanning.srv.MoveToHandPositionResponse()
                        spec = traj.GetConfigurationSpecification()
                        res.traj.joint_names = [str(j.GetName()) for j in robot.GetJoints(manip.GetArmIndices())]
                        
                        starttime = 0.0
                        PlayTrajectory(env, robot, traj, Tgoalee)
                        for i in range(traj.GetNumWaypoints()):
                            pt=trajectory_msgs.msg.JointTrajectoryPoint()
                            data = traj.GetWaypoint(i)
                            pt.positions = spec.ExtractJointValues(data,robot,manip.GetArmIndices(),0)                           
                                                      
                            starttime += spec.ExtractDeltaTime(data)
                            pt.time_from_start = rospy.Duration(starttime)
                            res.traj.points.append(pt)

                        print 'done'
                        return res

            finally:
                collisionmap.SendCommand("collisionstream 1")

        def MoveManipulatorFn(req):
            global options
            rospy.loginfo("MoveManipulator")
            try:
                with envlock:
                    if options.wait_for_collisionmap is not None:
                        starttime=time.time()
                        handgoalstamp = int64(req.hand_goal.header.stamp.to_nsec())
                        while True:
                            timepassed = time.time()-starttime
                            collisionstamp = collisionmap.SendCommand("gettimestamp")
                            if collisionstamp is not None:
                                if int64(collisionstamp)-handgoalstamp >= 0:
                                     break
                            if options.wait_for_collisionmap > 0 and timepassed > options.wait_for_collisionmap:
                                if options.simulation is not None:
                                    break;
                                raise ValueError('failed to acquire new collision map, collision timestamp is %s, service timestamp is %s'%(collisionstamp,handgoalstamp))
                            time.sleep(0.1) # wait

                    collisionmap.SendCommand("collisionstream 0")

                    with env:
                        basemanip = interfaces.BaseManipulation(robot,plannername=None if len(req.planner)==0 else req.planner,maxvelmult=options.maxvelmult)
                        
                        rospy.loginfo("MoveManipulator2")
                        manip = robot.SetActiveManipulator(req.manip_name)
                        if manip is None:
                            rospy.logerr('failed to find manipulator %s'%req.manip_name)
                            return None

                        try:
                            starttime = time.time()
                            traj = basemanip.MoveManipulator(goal=req.manip_goal,execute=False,outputtrajobj=True,maxiter=750,jitter=options.jitter)
                            rospy.loginfo('total planning time: %fs'%(time.time()-starttime))
                        except:
                            rospy.logerr('failed to solve for goal=%s, error messages are:'%repr(req.manip_goal))
                            return None
                        # parse trajectory data into the ROS structure
                        res = orrosplanning.srv.MoveManipulatorResponse()
                        spec = traj.GetConfigurationSpecification()
                        res.traj.joint_names = [str(j.GetName()) for j in robot.GetJoints(manip.GetArmIndices())]
                        starttime = 0.0
                        for i in range(traj.GetNumWaypoints()):
                            pt=trajectory_msgs.msg.JointTrajectoryPoint()
                            data = traj.GetWaypoint(i)
                            pt.positions = spec.ExtractJointValues(data,robot,manip.GetArmIndices(),0)
                            starttime += spec.ExtractDeltaTime(data)
                            pt.time_from_start = rospy.Duration(starttime)
                            res.traj.points.append(pt)
                            
                        return res

            finally:
                collisionmap.SendCommand("collisionstream 1")

        if options.request_for_joint_states == 'service':
            js = rospy.Service('SetJointState', orrosplanning.srv.SetJointState, UpdateRobotJointsFn)

        s1 = rospy.Service('MoveToHandPosition', orrosplanning.srv.MoveToHandPosition, MoveToHandPositionFn)
        s2 = rospy.Service('MoveManipulator', orrosplanning.srv.MoveManipulator, MoveManipulatorFn)
        RaveSetDebugLevel(DebugLevel.Debug)
        rospy.loginfo('openrave %s,%s service ready'%(s1.resolved_name,s2.resolved_name))

        p1 = rospy.Subscriber('/left_gripper_pose',geometry_msgs.msg.PoseStamped, PlanFromPose,'left')
        p2 = rospy.Subscriber('/right_gripper_pose',geometry_msgs.msg.PoseStamped, PlanFromPose,'right')
        if options.ipython:
            from IPython.Shell import IPShellEmbed
            ipshell = IPShellEmbed(argv='',banner = 'Dropping into IPython',exit_msg = 'Leaving Interpreter, back to program.')
            ipshell(local_ns=locals())
        else:
            rospy.spin()
    finally:
        RaveDestroy()



import rospy
import roslib
from openravepy import *
import copy
import ipdb



def retime_trajectory_with_end_effector_limits(robot, way_point_list, max_end_effector_dp, max_end_effector_da, end_effector_goal, max_joint_motion, max_end_effector_error):
    def goal_legal(current_config, goal_config, last_ee_tran):
        joint_diff = abs(current_config - goal_config)
        joint_violation = any(joint_diff > max_joint_motion)
        if joint_violation:
            return False
        with robot:
            robot.SetActiveDOFValues(goal_config)
            goal_ee_tran = robot.GetActiveManipulator().GetEndEffectorTransform()
            rel_tran = numpy.dot(numpy.linalg.inv(last_ee_tran), goal_ee_tran)
            dp = numpy.linalg.norm(rel_tran[:3,3])
            da = numpy.linalg.norm(axisAngleFromRotationMatrix(rel_tran))
            if dp > max_end_effector_dp or da > max_end_effector_da:
                return False
        return True

    def binary_goal_search(current_config,goal_config, last_ee_tran, config_list):
        b_current_config = copy.deepcopy(current_config)
        b_goal_config = copy.deepcopy(goal_config)
        b_last_ee_tran = copy.deepcopy(last_ee_tran)
        #ipdb.set_trace()
        while not goal_legal(b_current_config, b_goal_config, b_last_ee_tran):                   
            #config_list = [b_current_config] + config_list
            config_list, b_last_ee_tran = binary_goal_search(b_current_config, (b_goal_config - b_current_config)/2 + b_current_config, b_last_ee_tran, config_list)
            #config_list, b_last_ee_tran = binary_goal_search((b_goal_config - b_current_config)/2 + b_current_config, b_goal_config, b_last_ee_tran, config_list)
            
            b_current_config = config_list[-1]
            

        config_list.append(goal_config)
        robot.SetActiveDOFValues(goal_config)
        last_ee_tran = robot.GetActiveManipulator().GetEndEffectorTransform()
            
        return config_list, last_ee_tran
            
        # Insert time 0 if necessary
    traj_point_list = []
    if way_point_list[0] is not robot.GetActiveDOFValues():
        way_point_list.insert(0, robot.GetActiveDOFValues())

    last_ee_tran = robot.GetActiveManipulator().GetEndEffectorTransform()
    current_config = robot.GetActiveDOFValues()
    traj_point_list.append(current_config)
    for waypoint_num in xrange(len(way_point_list)):
        waypoint = way_point_list[waypoint_num]
        interpolated_traj, last_ee_tran = binary_goal_search(traj_point_list[-1], waypoint, last_ee_tran, [])

        traj_point_list += interpolated_traj
        #traj_point_list += [waypoint]
    
    abs(numpy.linalg.norm(last_ee_tran - end_effector_goal)) > max_end_effector_error
    return traj_point_list

def load_scene():
    global env
    global robot
    global traj
    env = Environment()
    env.Load('/home/jweisz/test_ws/src/drchubo/drchubo_v3/robots/drchubo_v3_no_solvers.robot.xml')
    robot = env.GetRobots()[0]
    f = open('/home/jweisz/.ros/test.traj')
    s = f.read()
    traj = RaveCreateTrajectory(env,'')
    traj.deserialize(s)
    robot.SetActiveDOFs([int(s) for s in  traj.GetConfigurationSpecification().GetGroupFromName('joint_values').name.split(' ')[2:]])
    return env, robot, traj



def traj_append(traj,waypt):
    """quickly append a waypoint to a trajectory"""
    n=traj.GetNumWaypoints()
    traj.Insert(n,waypt)

def create_trajectory(robot,waypts=None):
    """ Create a trajectory based on a robot's config spec. Optionally added a list of waypoints """
    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    config=robot.GetActiveConfigurationSpecification()
    #config.AddDeltaTimeGroup()
    traj.Init(config)

    if waypts is not None:
        raveLogInfo("Appending waypoint(s)")
        try:
            for w in waypts:
                traj_append(traj,w)
        except TypeError:
            #fallthrough if single waypoint
            traj_append(traj,waypts)

    return [traj,config]


def simple_retime(env, robot, traj, goal):

    way_point_list = [traj.GetWaypoint(n)[0:7] for n in xrange(traj.GetNumWaypoints())]
    interpolated_traj = retime_trajectory_with_end_effector_limits(robot, way_point_list, .05, .1, goal, .1, None)
    robot.SetActiveDOFs([int(s) for s in  traj.GetConfigurationSpecification().GetGroupFromName('joint_values').name.split(' ')[2:]])    
    traj2 = create_trajectory(robot, interpolated_traj)
    
    return traj2[0]
    
def test_retimer():
    global env
    global robot
    global traj
    if not 'env' in globals() or not 'traj' in globals() or not 'robot' in globals():
        load_scene()
    
    way_point_list = [traj.GetWaypoint(n)[0:7] for n in xrange(traj.GetNumWaypoints())]
    interpolated_traj = retime_trajectory_with_end_effector_limits(robot, way_point_list, .05, .1, None, .1, None)
    
    
    return interpolated_traj



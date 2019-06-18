from geometry_msgs.msg import Pose

from agentThread import AgentThread
from gvh import Gvh
from CyPyHous3.src.All import PlannedPath


class BasicFollowApp(AgentThread):

    def __init__(self, pid: int, num_bots: int):
        super(BasicFollowApp, self).__init__(Gvh(pid, num_bots))
        self.start()



    def run(self):
        dest1 = Pose()
        dest1.position.x, dest1.position.y, dest1.position.z = 0., 0., 1.
        dest2 = Pose()
        dest2.position.x, dest2.position.y, dest2.position.z = 0., 0., 0.
        planner = PlannedPath.Planner(self.agent_gvh.moat)


        while not self.stopped():


            pp = planner.planned_path((self.agent_gvh.moat.position.x,self.agent_gvh.moat.position.y),(dest1.position.x, dest1.position.y))
            planner.follow_path(pp)



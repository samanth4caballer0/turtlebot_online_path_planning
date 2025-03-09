import rospy
from std_srvs.srv import Trigger, TriggerRequest
import py_trees


# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        rospy.wait_for_service("/manage_objects/check_object")
        try:
            self.server = rospy.ServiceProxy("/manage_objects/check_object", Trigger)
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name
            )
        except rospy.ServiceException as e:
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/check_object".format(self.name)
            )
            resp = self.server(TriggerRequest())
            if resp.success:
                self.blackboard.object_name = resp.message
                print("set to blackboard: ", resp.message)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/check_object".format(
                    self.name
                )
            )
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [CheckObject::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )

from rospy.rostime import Time
from rospy import ServiceException

from python_qt_binding.QtCore import QObject
from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from arni_core.host_lookup import HostLookup
from arni_core.helper import SEUID
import arni_core.helper as helper
from arni_msgs.srv import NodeReaction


class NodeItem(AbstractItem):
    """A NodeItem represents a node with all of its data. It also has a interface to start/stop/restart nodes."""


    def __init__(self, logger, seuid, parent=None):
        """
        Initializes the NodeItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        #add the content
        AbstractItem.__init__(self, logger, seuid, parent)
        #super(NodeItem, self).__init__(seuid, parent)
        self._type = "node"
        self.__parent = parent

        self._type = "node"

        self._attributes = []
        self._attributes.extend(["node_cpu_usage_mean", "node_cpu_usage_stddev", "node_cpu_usage_max",
                      "node_cpu_usage_core_mean",
                      "node_cpu_usage_core_stddev", "node_cpu_usage_core_max", "node_gpu_usage_mean", "node_gpu_usage_stddev",
                      "node_gpu_usage_max", "node_ramusage_mean", "node_ramusage_stddev", "node_ramusage_max",
                      "node_message_frequency_mean", "node_message_frequency_stddev", "node_message_frequency_max", "node_bandwidth_mean", "node_bandwidth_stddev",
                      "node_bandwidth_max", "node_write_mean", "node_write_stddev", "node_write_max", "node_read_mean",
                      "node_read_stddev", "node_read_max"])

        for item in self._attributes:
            self._add_data_list(item)

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        del self._attributes

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new NodeItem")


    def execute_action(self, action):
        """
        Sends a signal to top or restart the node.

        :param action: action to be executed
        :type action: RemoteAction
        """
        #create the name of the service
        #service_name = "/execute_node_reaction/" +
        #rospy.wait_for_service(
        
        host_formatted = helper.underscore_ip(self.__parent.get_seuid()[2:])
        service_name = "/execute_node_reaction/%s" % host_formatted
        try:            
            execute = rospy.ServiceProxy(
                service_name, NodeReaction)
            resp = execute(self.seuid[2:], action, '')
            #rospy.logdebug(
                #"sending command '%s' to node %s returned: %s"
                #% (self.__command, self._node, resp.returnmessage))
            print resp.returnmessage
        except ServiceException:
            rospy.logdebug(
                "could not stop node %s, service %s not found"
                % (self.seuid, service_name))

    def get_detailed_data(self):
        """
        Returns the detailed data of the NodeItem.
        
        :returns: str
        """
        #todo: fill the content sensefully!
        data_dict = self.get_latest_data()

        content = "<p class=\"detailed_data\">"


        content += QTranslator.translate("AbstractItem", "node_cpu_usage_mean") + ": " + str(data_dict["node_cpu_usage_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_cpu_usage_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_cpu_usage_stddev") + ": " + str(data_dict["node_cpu_usage_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_cpu_usage_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_cpu_usage_max") + ": " + str(data_dict["node_cpu_usage_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_cpu_usage_max_unit") + " <br>"
        for i in data_dict["node_cpu_usage_core_mean"]:
            content += QTranslator.translate("AbstractItem", "core" + str(i + 1)) + "<br>"
            content += QTranslator.translate("AbstractItem", "node_cpu_usage_core_mean") + ": " + str(data_dict["node_cpu_usage_core_mean"]) \
                       + " " + QTranslator.translate("AbstractItem", "node_cpu_usage_core_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "node_cpu_usage_core_stddev") + ": " + str(data_dict["node_cpu_usage_core_stddev"]) \
                       + " " + QTranslator.translate("AbstractItem", "node_cpu_usage_core_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "node_cpu_usage_core_max") + ": " + str(data_dict["node_cpu_usage_core_max"]) \
                       + " " + QTranslator.translate("AbstractItem", "node_cpu_usage_core_max_unit") + " <br>"
        for i in data_dict["node_gpu_usage_mean"]:
            content += QTranslator.translate("AbstractItem", "node_gpu_usage_mean") + ": " + str(data_dict["node_gpu_usage_mean"]) \
                       + " " + QTranslator.translate("AbstractItem", "node_gpu_usage_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "node_gpu_usage_stddev") + ": " + str(data_dict["node_gpu_usage_stddev"]) \
                       + " " + QTranslator.translate("AbstractItem", "node_gpu_usage_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "node_gpu_usage_max") + ": " + str(data_dict["node_gpu_usage_max"]) \
                       + " " + QTranslator.translate("AbstractItem", "node_gpu_usage_max_unit") + " <br>"

        content += QTranslator.translate("AbstractItem", "node_ramusage_mean") + ": " + str(data_dict["node_ramusage_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_ramusage_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_ramusage_stddev") + ": " + str(data_dict["node_ramusage_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_ramusage_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_ramusage_max") + ": " + str(data_dict["node_ramusage_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_ramusage_max_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_message_frequency_mean") + ": " + str(data_dict["node_message_frequency_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_message_frequency_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_message_frequency_stddev") + ": " + str(data_dict["node_message_frequency_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_message_frequency_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_message_frequency_max") + ": " + str(data_dict["node_message_frequency_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_message_frequency_max_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_bandwidth_mean") + ": " + str(data_dict["node_bandwidth_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_bandwidth_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_bandwidth_stddev") + ": " + str(data_dict["node_bandwidth_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_bandwidth_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_bandwidth_max") + ": " + str(data_dict["node_bandwidth_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_bandwidth_max_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_write_mean") + ": " + str(data_dict["node_write_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_write_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_write_stddev") + ": " + str(data_dict["node_write_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_write_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_write_max") + ": " + str(data_dict["node_write_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_write_max_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_read_mean") + ": " + str(data_dict["node_read_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_read_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_read_stddev") + ": " + str(data_dict["node_read_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_read_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "node_read_max") + ": " + str(data_dict["node_read_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "node_read_max_unit") + " <br>"

        content += "</p>"
        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["node_cpu_usage_mean", "node_bandwidth_mean"]

    def get_short_data(self):
        return "NodeItem"

    def can_execute_actions(self):
        """
        This item can execute actions, so it returns True

        :return: True
        """
        return True

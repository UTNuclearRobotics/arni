class StorageContainer:

    def __init__(self, timestamp, identifier, data_raw, data_rated):
        """
        Creates a storage container for raw and rated metadata.
        :param timestamp: The timestamp of capture of the raw data.
        :type timestamp: rospy.Time.
        :param identifier: The seuid identifying the source of the data.
        :type identifier: str.
        :param data_raw: The raw data as the source sent it.
        :param data_rated: The rated data.
        :type data_rated: RatedStatistics.
        """
        self.timestamp = timestamp
        self.identifier = identifier
        self.data_raw = data_raw
        self.data_rated = data_rated
import rospy

class DataManager:
    def __init__(self):
        self.data = {}
        self.param_prefix = "/vision_system/data/"

    def update(self, key, value, operation="set"):
        """
        Update data.
        operation: "set", "add", "append"
        """
        current_val = self.data.get(key)
        
        if operation == "set":
            new_val = value
        elif operation == "add":
            if current_val is None: current_val = 0
            new_val = current_val + value
        elif operation == "append":
            if current_val is None: current_val = []
            if not isinstance(current_val, list): current_val = [current_val]
            current_val.append(value)
            new_val = current_val
        else:
            rospy.logwarn(f"Unknown operation: {operation}")
            return

        self.data[key] = new_val
        rospy.set_param(self.param_prefix + key, new_val)
        rospy.loginfo(f"Data updated: {key} = {new_val}")

    def get(self, key, default=None):
        return self.data.get(key, default)

    def get_from_param(self, key, default=None):
        return rospy.get_param(self.param_prefix + key, default)

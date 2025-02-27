import pandas as pd
import threading

class DataManager:
    def __init__(self):
        """Manages storage and retrieval of ROS topic data using Pandas."""
        self.lock = threading.Lock()
        self.data = {
            "pose": pd.DataFrame(columns=["timestamp", "x", "y", "z", "roll", "pitch", "yaw"]),
            "twist": pd.DataFrame(columns=["timestamp", "vx", "vy", "vz"]),
            "current": pd.DataFrame(columns=["timestamp", "value"]),
            "voltage": pd.DataFrame(columns=["timestamp", "value"]),
            "temperature": pd.DataFrame(columns=["timestamp", "value"]),
            "pressure": pd.DataFrame(columns=["timestamp", "value"]),
        }

    def add_data(self, topic: str, values: dict):
        """Stores new data for a given topic in Pandas DataFrame."""
        with self.lock:
            if topic in self.data:
                df = self.data[topic]
                self.data[topic] = pd.concat([df, pd.DataFrame([values])], ignore_index=True)

    def get_latest(self, topic: str):
        """Returns the latest recorded value of a topic."""
        with self.lock:
            if topic in self.data and not self.data[topic].empty:
                return self.data[topic].iloc[-1].to_dict()
            return None

    def get_data(self, topic: str, limit=100):
        """Returns the most recent `limit` number of entries for a topic."""
        with self.lock:
            if topic in self.data:
                return self.data[topic].tail(limit)
            return pd.DataFrame()

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

bag_path = 'Dataset/bags/0303_clearwater_yellowtable/0001'

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

count = 0
while reader.has_next():
    (topic, data, t) = reader.read_next()
    if 'joint_states' in topic and topic_types[topic] == 'sensor_msgs/msg/JointState':
        msg = deserialize_message(data, JointState)
        print(f"Topic: {topic}")
        print(f"Joint Names: {msg.name}")
        count += 1
        if count >= 2: # Check first 2 messages (likely one left, one right)
            break

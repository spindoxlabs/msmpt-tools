import threading
from rexasi_tracker_msgs.msg import Tracks

class TimeSequencer:
    """
    Buffers messages and calls the callback in timestamp order.
    slop: seconds to wait for possible out-of-order messages.
    queue_size: max number of messages to buffer.
    """
    def __init__(self, node, input_topic, callback, slop=0.1):
        self.callback = callback
        self.slop = slop
        self.logger = node.get_logger()
        self.buffers = {}
        self.last_message = []
        self.num_messages = 0

        self.tracks_sub = node.create_subscription(
            Tracks,
            input_topic,
            self.message_received,
            100
        )
        self.lock = threading.Lock()
        self.received_message_event = threading.Event()
        self.thread = threading.Thread(target=self._run)
        self.thread.start()
        
    def message_received(self, msg):
        with self.lock:
            self.logger.debug(f"Entering callback")
            self.logger.debug(f"Callback is processing message from sensor {msg.sensor_id} with timestamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            self.last_message.append(msg)
            self.received_message_event.set()

    def process_messages(self):
        while (len(self.last_message) > 0):
            message_to_process = self.last_message.pop(0)
            self.logger.debug(f"Received message from sensor {message_to_process.sensor_id}, current buffer size: {self.num_messages}")
            sensor_id = message_to_process.sensor_id
            if sensor_id not in self.buffers:
                self.buffers[sensor_id] = []
            self.buffers[sensor_id].append(message_to_process)
            self.num_messages += 1

        #exist_empty_buffer, message, sensor = self._get_older_message()
        return self._get_oldest_message()         

    # Returns True if m1 is older than m2, False otherwise
    def _is_older(self, m1, m2):
        if (m1.header.stamp.sec < m2.header.stamp.sec) or (
            (m1.header.stamp.sec == m2.header.stamp.sec and m1.header.stamp.nanosec < m2.header.stamp.nanosec)):
            return True
        else:
            return False    
        
    # Returns a tuple:
    # - the first element is True if all buffers have at least one message
    #
    def _get_oldest_message(self):
        oldest_message = None
        sensor_message = None
        exist_empty_buffer = False
        for sensor, buffer in self.buffers.items():
            if len(buffer) > 0:
                message = buffer[0]
                if oldest_message is None or self._is_older(message, oldest_message):
                    sensor_message = sensor
                    oldest_message = message
            else:
                exist_empty_buffer = True

                
        return exist_empty_buffer, oldest_message, sensor_message
    
    def send_all_messages(self):
        while (self.num_messages > 0):
            _, message, sensor = self._get_oldest_message()
            self.buffers[sensor].pop(0)
            self.send_message(message)            

    def send_message(self, message):
        self.callback(message)
        self.num_messages -= 1

    def _run(self):
        cycle_count = 0
        self.logger.debug("TimeSequencer started")
        while True:
            self.logger.debug(f"\ncycle {cycle_count}: Waiting for messages, current buffer size: {self.num_messages}")
            if (self.received_message_event.wait(timeout=self.slop)):
                with self.lock:
                    self.received_message_event.clear()
                    exist_empty_buffer, message, sensor = self.process_messages()
                    self.logger.debug(f"cycle {cycle_count}:exist_empty_buffer: {exist_empty_buffer}, sensor: {sensor}")
                    if not exist_empty_buffer:
                        while not exist_empty_buffer:
                            self.logger.debug(f"cycle {cycle_count}:Sending message from sensor {sensor} with timestamp {message.header.stamp.sec}.{message.header.stamp.nanosec}")
                            self.buffers[sensor].pop(0)  # Remove the oldest message from the buffer
                            self.send_message(message)
                            exist_empty_buffer, message, sensor = self._get_oldest_message()
                            self.logger.debug(f"cycle {cycle_count}:exist_empty_buffer: {exist_empty_buffer}, sensor: {sensor}")
                    else: 
                        self.logger.debug(f"cycle {cycle_count}:at least one buffer is empty, waiting for more messages")
            else:
                self.logger.debug(f"cycle {cycle_count}:No messages received in the last {self.slop} seconds, sending all messages")
                with self.lock:
                    self.send_all_messages()
                self.logger.debug(f"cycle {cycle_count}:All messages sent, current buffer size: {self.num_messages}")

            for sid, buffer in self.buffers.items():
                self.logger.debug(f"cycle {cycle_count}:Buffer for sensor {sid} has {len(buffer)} messages")
            cycle_count += 1


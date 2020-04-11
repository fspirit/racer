import rclpy

from rclpy.node import Node
from rclpy.timer import Timer
from racer_msgs.msg import Actuators
from racer_msgs.msg import RecordingStatus

from jetcam.csi_camera import CSICamera

class Factory():

    def create_timer(self, callback, timer_period_ns):
        return Timer(callback = callback, timer_period_ns=timer_period_ns)

    def create_camera(self, width, height, capture_width, capture_height, capture_fps):
        return CSICamera(width=width, height=height, capture_width=capture_width, capture_height=capture_height, capture_fps=capture_fps)


class IdleState():    

    def handle_actuators(self, actuators):
        pass

    def next_state(self, recording_status, states):
        if recording_status.is_on:
            return states['recording']
        return self


class RecordingState():

    def __init__(self, factory, settings):
        self.recording_started_ = False
        self.factory_ = factory
        self.fps_ = settings['camera_fps']

    def handle_actuators(self, actuators):        
        self.last_actuators_ = actuators
        
    def start_recording(self):        
        
        self.camera_ = self.factory_.create_camera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=self.fps_)
        self.timer_ = self.factory_.create_timer(self.save_frame, (1.0 / self.fps_ * 1e9))        

        self.recording_started_ = True

    def save_frame(self):
        image = self.camera_.read()
        logger = rclpy.logging.get_logger('RecordingState')
        logger.info("Save frame called")

        # put to numpy file with along with actuators

    def finish_recording(self):
        self.camera_ = None
        self.timer_ = None

        self.recording_started_ = False

    def next_state(self, recording_status, states):
        if not recording_status.is_on:
            self.finish_recording()
            return states['idle']
        elif not self.recording_started_:
            self.start_recording()
            return self
        else:
            return self


class ILDatasetRecorder(Node):

    def __init__(self, factory, settings = {}):
        super().__init__('il_dataset_recorder')

        self.states_ = {'idle': IdleState(), 
            'recording': RecordingStatus(factory, settings)}
        self.current_state_ = self.states_['idle']       

        self.actuators_subscription = self.create_subscription(
            Actuators,
            'actuators',
            self.handle_actuators,
            10)

        self.recording_command_subscription = self.create_subscription(
            RecordingStatus,
            'recording_status',
            self.handle_recording_command,
            10)

    def handle_actuators(self, msg):
        self.current_state_.handle_actuators(msg)

    def handle_recording_command(self, recording_status):
        self.current_state_ =  self.state_.next_state(recording_status, self.states_)

def main(args=None):
    rclpy.init(args=args)

    factory = Factory()
    settings = {'camera_fps': 2}

    il_dataset_recorder = ILDatasetRecorder(factory, settings)

    rclpy.spin(il_dataset_recorder)

    il_dataset_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
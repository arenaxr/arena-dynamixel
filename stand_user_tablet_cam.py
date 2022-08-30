from kubi_wrapper import *
from arena import *
import random
import scipy
import math


kubi_1 = Dynamixel_Servo('/dev/tty.usbserial-FT6RW6MQ')
kubi_2 = Dynamixel_Servo('/dev/tty.usbserial-FT6RWE8K')


kubi_1.connect_Dynamixel()
kubi_1.init_Dynamixel()


''' Camera-Dynamixel Sync
'''
MIN_DISPLACEMENT = 0.5
LINE_TTL = 5
HALF_REVOLUTION = 180

class CameraState(Object):
    def __init__(self, camera):
        self.camera = camera
        self.prev_pos = None
        self.line_color = Color(
                random.randint(0,255),
                random.randint(0,255),
                random.randint(0,255)
            )

    @property
    def curr_pos(self):
        return self.camera.data.position

    @property
    def id(self):
        return self.camera.object_id

    @property
    def curr_rot(self):
        return self.camera.data.rotation
    
    @property
    def displ_name(self):
        return self.camera.displayName

    @property
    def displacement(self):
        if self.prev_pos:
            return self.prev_pos.distance_to(self.curr_pos)
        else:
            return 0

cam_states = []


def user_join_callback(scene, cam, msg):
    global cam_states

    cam_state = CameraState(cam)
    cam_states += [cam_state]

    

scene = Scene(host="arena-dev1.conix.io", scene="first_playground")
scene.user_join_callback = user_join_callback

def rotation_quat2euler(quat):
        rotq = scipy.spatial.transform.Rotation.from_quat(list(quat))
        # do not change to xzy, weird things happen
        return tuple(rotq.as_euler('xyz', degrees=True))



user_cam_dict = {}
user_entered  = False

def cam_create(cam_state_id):
    global user_cam_dict
    global user_entered

    if not user_entered or cam_state_id in user_cam_dict or user_cam_dict:
        return

    new_cam = Object(**{"object_id": "fixed_cam", "camera":{"active": True}, "position": (0, 0, -0.53), "rotation": (0, 180, 0), "parent": cam_state_id })

    user_cam_dict[cam_state_id] = new_cam
    print(user_cam_dict[cam_state_id])

    scene.add_object(new_cam)

call_index = 0
rotation_x = 0
rotation_y = 0
@scene.run_forever(interval_ms=5)
def cam_motor_sync():
    global call_index
    global rotation_x
    global rotation_y
    global user_entered

    vid_ball = scene.all_objects["video_ball"]
    
    call_index += 1
    if(call_index % 15 == 0):
        for cam_state in cam_states:
            dx = vid_ball.data.position.x - cam_state.curr_pos.x
            dz = vid_ball.data.position.z - cam_state.curr_pos.z
            disp = math.sqrt(dx**2 + dz**2)
            if vid_ball.data.radius < disp:
                user_entered = False
                continue
            user_entered = True
            cam_create(cam_state.id)

            # DO NOT CHANGE coordinate ordering
            euler_cords = rotation_quat2euler((cam_state.curr_rot.y, 
            cam_state.curr_rot.x, cam_state.curr_rot.z, cam_state.curr_rot.w))
        
            rotation_x = euler_cords[1]
            rotation_y = euler_cords[0]
    
            # if kubi_1.occupant != "" and kubi_1.occupant != cam_state.id:
            #     continue
            
            # if kubi_1.occupant == "":
            #     kubi_1.occupant = cam_state.id
            

    if not user_entered:
        return

    print(user_entered)
    kubi_1.pan_To_Angle(rotation_y)
    kubi_1.tilt_To_Angle(rotation_x)
      
scene.run_tasks() # will block

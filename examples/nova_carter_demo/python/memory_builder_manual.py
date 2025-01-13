import time
from remembr.memory.memory import MemoryItem
from remembr.memory.milvus_memory import MilvusMemory

import numpy as np
from scipy.spatial.transform import Rotation as R


memory = MilvusMemory("video_pos", "127.0.0.1")
memory.reset()

# captions = [
#     "(   0,   0) The video shows an elevator. There are plants next to it", 
#     "( 100, 100) The video shows a restroom.",
#     "( 200, 200) The video shows a kitchen. There is a fridge and a water dispenser.",
#     "( 300, 300) The video shows a meeting room. A 201 sign is on the door.",
#     "( 400, 400) The video shows a cafe.",
#     "( 500, 500) The video shows a hallway and there is no people.",
#     "( 600, 600) The video shows a meeting room. A 203 sign is on the door.",
#     "( 700, 700) The video shows several trash cans with diffent color.",
#     "( 800, 800) The video shows a vending machine with a fire extinguisher.",
#     "( 900, 900) The video shows many tables and chairs in a open space.",
#     "(1000,1000) The video shows a meeting room. A 205 sign is on the door.",
# ]


# for idx, cap in enumerate(captions):
#     item = MemoryItem(
#         caption=cap,
#         time=time.time()+idx,
#         position=[idx*100, idx*100, 1.57],
#         theta=1.57
#     )
#     memory.insert(item)

captions = [
    ["The video shows many tables and chairs in a open space.",[15.160, 3.231, 0.0], [0.0, 0.0, -0.875, 0.483]],
    ["The video shows a meeting room. A 205 sign is on the door.",[6.805, -2.013, 0.0], [0.0, 0.0, -0.958, 0.284]],
    ["The video shows a cafe.",[2.647, 6.585, 0.0], [0.0, 0.0, 0.180, 0.983]],
    ["The video shows a restroom.",[1.596, 13.409, 0.0], [0.0, 0.0, 0.979, 0.201]],
    ["The video shows a kitchen. There is a fridge.",[-2.514, 15.763, 0.0], [0.0, 0.0, 0.084, 0.996]],
    ["The video shows a vending machine which sells drinks.",[-5.250, 16.084, 0.0], [0.0, 0.0, 0.863, 0.505]],
    ["The video shows an elevator. There are plants next to it",[4.449, 13.153, 0.0], [0.0, 0.0, 0.433, 0.901]],
]

for idx, (cap, pos, quat) in enumerate(captions):
    euler_rot_z = R.from_quat(np.array(quat)).as_euler('xyz')[-1] # take z rotation

    item = MemoryItem(
        caption=cap,
        time=time.time()+idx*10,
        position=pos,
        theta=euler_rot_z
    )
    memory.insert(item)
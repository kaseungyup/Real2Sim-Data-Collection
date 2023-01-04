import math
import numpy as np
from classes.timer import Timer

def quaternion_to_vector(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z

def get_traj(qpos):
    qpos.T[1] = -qpos.T[1]
    qpos.T[3] = -qpos.T[3]
    qpos.T[5] = -qpos.T[5]
    qpos.T[7] = -qpos.T[7]
    qpos = 2048/180*qpos+2048

    # 41 degree 
    plus_joint_limit = 2048/180*41 + 2048
    minus_joint_limit = -2048/180*41 + 2048
    qpos[:, 0] = np.clip(qpos[:, 0], minus_joint_limit, plus_joint_limit)
    qpos[:, 2] = np.clip(qpos[:, 2], minus_joint_limit, plus_joint_limit)
    qpos[:, 4] = np.clip(qpos[:, 4], minus_joint_limit, plus_joint_limit)
    qpos[:, 6] = np.clip(qpos[:, 6], minus_joint_limit, plus_joint_limit)
    return qpos


def run_snapbot(qpos, snapbot, Hz, max_sec):
    traj = get_traj(qpos)
    t = Timer(_HZ=Hz, _MAX_SEC=max_sec)
    t.start()
    idx, threshold= 0, 0
    flag = False
    while t.is_notfinished():
        if t.do_run():
            pos = traj[idx]
            if not flag:
                flag = True
                threshold = pos
            idx = idx + 1
            snapbot.set_goalpos((np.array(pos, dtype=np.int32).tolist()))
            if idx == traj.shape[0]:
                t.finish()
    print("FINISHED")

def run_snapbot_single(qpos, snapbot):
    qpos.T[1] = -qpos.T[1]
    qpos.T[3] = -qpos.T[3]
    qpos.T[5] = -qpos.T[5]
    qpos.T[7] = -qpos.T[7]
    qpos = 2048/180*qpos+2048

    plus_joint_limit = 2048/180*41 + 2048
    minus_joint_limit = -2048/180*41 + 2048
    qpos[0] = np.clip(qpos[0], minus_joint_limit, plus_joint_limit)
    qpos[2] = np.clip(qpos[2], minus_joint_limit, plus_joint_limit)
    qpos[4] = np.clip(qpos[4], minus_joint_limit, plus_joint_limit)
    qpos[6] = np.clip(qpos[6], minus_joint_limit, plus_joint_limit)

    traj = qpos
    threshold = 0
    flag = False
    pos = traj
    if not flag:
        flag = True
        threshold = pos
    snapbot.set_goalpos((np.array(pos, dtype=np.int32)))



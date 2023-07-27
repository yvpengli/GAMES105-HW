from scipy.spatial.transform import Rotation as R
from Lab1_FK_answers import *
from task1_forward_kinematics import *

def fun(list):
    list.append(122)

if __name__ == "__main__":
    bvh_file_path = "data/walk60.bvh"

    a = [1 , 2, 3]
    b = [4, 5]
    a.append(b[:])
    print(a)

    # part2_one_pose(None, bvh_file_path)

# if __name__ == "__main__":
#     print("hello Test")
#     quat = R.from_rotvec([0, 3, 2]).as_quat()
#     euler_angle = R.from_matrix([[-0.89428806, 0.24822376, -0.37233564],
#                                  [-0.24822376, 0.41714214, 0.8742868],
#                                  [0.37233564, 0.8742868, -0.31143019]]).as_euler('XYZ', degrees=True)
#     matrix = R.from_quat([0., 0.80976237, 0.53984158, -0.22990426]).as_matrix()
#
#     a = R.from_rotvec([0, 3, 2])
#     b = R.from_rotvec([2, 1, 5])
#     c = R.from_rotvec([4, 7, 3])
#     ans1 = (a*b).as_matrix()
#     ans2 = a.as_matrix() @ b.as_matrix()
#     ans3 = a.as_matrix().dot(b.as_matrix())
#
#     print(ans1 == ans2)
#     print(ans1)
#     print(ans2)
#     print(ans3)
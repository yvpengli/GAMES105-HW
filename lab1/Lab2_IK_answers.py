import numpy as np
from scipy.spatial.transform import Rotation as R

def rotation_matrix(a, b):
    a=a/np.linalg.norm(a)
    b=b/np.linalg.norm(b)
    n = np.cross(a, b)
    # 旋转矩阵是正交矩阵，矩阵的每一行每一列的模，都为1；并且任意两个列向量或者任意两个行向量都是正交的。
    # 计算夹角
    cos_theta = np.dot(a, b)
    sin_theta = np.linalg.norm(n)
    theta = np.arctan2(sin_theta, cos_theta)
    # 构造旋转矩阵
    c = np.cos(theta)
    s = np.sin(theta)
    v = 1 - c
    rotation_matrix = np.array([[n[0]*n[0]*v+c, n[0]*n[1]*v-n[2]*s, n[0]*n[2]*v+n[1]*s],
                                 [n[0]*n[1]*v+n[2]*s, n[1]*n[1]*v+c, n[1]*n[2]*v-n[0]*s],
                                 [n[0]*n[2]*v-n[1]*s, n[1]*n[2]*v+n[0]*s, n[2]*n[2]*v+c]])
    return rotation_matrix

def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):

    def get_joint_rotations():
        joint_rotations = np.empty(joint_orientations.shape)
        for i in range(len(joint_name)):
            if joint_parent[i] == -1:
                joint_rotations[i] = R.from_euler('XYZ', [0.,0.,0.]).as_quat()
            else:
                joint_rotations[i] = (R.from_quat(joint_orientations[joint_parent[i]]).inv() * R.from_quat(joint_orientations[i])).as_quat()
        return joint_rotations

    def get_joint_offsets():
        joint_offsets = np.empty(joint_positions.shape)
        for i in range(len(joint_name)):
            if joint_parent[i] == -1:
                joint_offsets[i] = np.array([0.,0.,0.])
            else:
                joint_offsets[i] = joint_initial_position[i] - joint_initial_position[joint_parent[i]]
        return joint_offsets

    def quat_mul( a, b ):
        return (R.from_quat(a) * R.from_quat(b)).as_quat()

    joint_name = meta_data.joint_name
    joint_parent = meta_data.joint_parent
    joint_initial_position = meta_data.joint_initial_position

    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()

    # print(path)
    # print(path_name)
    # print(path1)
    # print(path2)

    joint_rotations = get_joint_rotations()
    joint_offsets = get_joint_offsets()

    times = 10
    # end_idx表示手部位置
    end_idx = path[-1]
    while times > 0:
        times -= 1
        distance = np.linalg.norm(joint_positions[end_idx] - target_pose)
        if distance < 0.001:
            break
        # 先处理path1, 从手部到根节点(不包括根节点)
        for i in range(1, len(path1)):
            joint_idx = path1[i]
            vec_to_end=joint_positions[end_idx]-joint_positions[joint_idx]
            vec_to_target=target_pose-joint_positions[joint_idx]
            delta_rotation = R.from_matrix( rotation_matrix(vec_to_end,vec_to_target))

            # 更新当前节点的 orientation
            joint_orientations[joint_idx] = (delta_rotation * R.from_quat(joint_orientations[joint_idx])).as_quat()
            joint_rotations[joint_idx] = (delta_rotation * R.from_quat(joint_rotations[joint_idx])).as_quat()
            # 更新后面节点的 orientation 和 position
            for j in range(i-1, -1, -1):
                child_idx = path1[j]
                before_idx = path1[j+1]
                joint_orientations[child_idx] = (delta_rotation * R.from_quat(
                    joint_orientations[child_idx])).as_quat()
                joint_positions[child_idx] = joint_positions[before_idx] + R.from_quat(joint_orientations[before_idx]).as_matrix() @ joint_offsets[child_idx]

        # 处理path2,从脚到根节点, 倒着遍历, 从根节点后面的第一个节点开始
        for i in range(len(path2)-2, 0, -1):
            joint_idx = path2[i]
            p_idx = path2[i+1]
            # 通过joint_idx计算旋转, 但是修改的是p_idx的旋转和位移
            vec_to_end = joint_positions[end_idx] - joint_positions[joint_idx]
            vec_to_target = target_pose - joint_positions[joint_idx]
            delta_rotation = R.from_matrix(rotation_matrix(vec_to_end, vec_to_target))

            for j in range(i+1, len(path2)):
                child_idx = path2[j]
                before_idx = path2[j-1]
                joint_orientations[child_idx] = (delta_rotation * R.from_quat(joint_orientations[child_idx])).as_quat()
                joint_positions[child_idx] = joint_positions[before_idx] + R.from_quat(joint_orientations[child_idx]).as_matrix() @ (-joint_offsets[before_idx])

            for j in range(len(path1)-1, -1, -1):
                child_idx = path1[j]
                before_idx = joint_parent[child_idx]
                joint_orientations[child_idx] = (delta_rotation * R.from_quat(
                    joint_orientations[child_idx])).as_quat()
                joint_positions[child_idx] = joint_positions[before_idx] + R.from_quat(
                    joint_orientations[before_idx]).as_matrix() @ joint_offsets[child_idx]

    for i in range(len(joint_orientations)):
        if i in path:
            continue
        p = joint_parent[i]
        joint_orientations[i] = quat_mul(joint_rotations[i], joint_orientations[p])
        joint_positions[i] = joint_positions[p] + R.from_quat(joint_orientations[p]).as_matrix() @ joint_offsets[i]

    return joint_positions, joint_orientations


def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    
    return joint_positions, joint_orientations

def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    
    return joint_positions, joint_orientations
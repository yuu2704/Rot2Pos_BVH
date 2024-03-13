import glob
import argparse
import os
import copy
import numpy as np
from scipy.spatial.transform import Rotation

class BvhNode:
    def __init__(self, name, parent=None):
        self.name = name
        self.offset = []
        self.channels = []
        self.motion = []
        self.positions = None
        self.children = []
        self.parent = parent
        if self.parent:
            self.parent.add_child(self)

    def add_child(self, item):
        item.parent = self
        self.children.append(item)

    def add_motion(self, data):
        self.motion.append(data)

    def print_tree(self):
        print(self.name)
        for child in self.children:
            child.print_tree()

    def calculate_positions(self, std_pos, std_rot, frame, position_data):
        euler = ''.join([channel[0] for channel in self.channels[3:]])
        print(self.name, self.motion[frame][3:])
        euler_rot = np.array(self.motion[frame][3:])
        diff_rot = Rotation.from_euler(euler, euler_rot, degrees=True)
        diff_pos = std_rot @ self.motion[frame][:3].T
        pos = std_pos+diff_pos
        rot = std_rot @ diff_rot.as_matrix()
        position_data[self.name] = pos
        for node in self.children:
            position_data = node.calculate_positions(pos, rot, frame, position_data)
        return position_data


class Bvh:
    def __init__(self,path):
        self.path = path
        self.frame = 0
        self.frame_time = 0
        self.root = self.load_bvh(path)

    def load_bvh(self, path):
        with open(path, 'r') as f:
            lines = f.readlines()
            datas = []
            for line in lines:
                line = line.rstrip('\n')
                data = line.split()
                datas.append(data)
            node_stack = []
            node_list = []
            frame_time_found = False
            node = None
            end_node = False
            for data in datas:
                if not frame_time_found:
                    if data[0]=='ROOT':
                        node = BvhNode(data[1])
                    elif data[0]=='JOINT':
                        node = BvhNode(data[1], node_stack[-1])
                    elif data[0]=='{':
                        if not end_node:
                            node_stack.append(node)
                            node_list.append(node)
                    elif data[0]=='}':
                        if not end_node:
                            node_stack.pop()
                        end_node = False
                    elif data[0]=='CHANNELS':
                        node.channels=data[2:]
                    elif data[0]=='End':
                        end_node=True
                    elif data[0]=='OFFSET':
                        if not end_node:
                            node.offset=np.array(list(map(float, data[1:])))
                    elif data[0]=='Frames:':
                        self.frame = int(data[1])
                    elif data[0]=="Frame" and data[1]=="Time:":
                        self.frame_time = float(data[2])
                        frame_time_found = True
                else:
                    data = list(map(float, data))
                    idx = 0
                    print(len(node_list))
                    for i in range(len(node_list)):
                        node_list[i].motion.append(np.array(data[idx:idx+len(node.channels)]))
                        idx+=len(node_list[i].channels)
        # node_list[0].print_tree()
        # print(node_list[1].motion)
        return node_list[0]


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root_path', type=str)
    parser.add_argument('--save_path', type=str)
    args = parser.parse_args()
    return args

def hip_transration(root_path, save_path):
    path_list = glob.glob(os.path.join(root_path, '*.bvh'))
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    print(f'{len(path_list)} files')
    for path in path_list:
        bvh = Bvh(path)
        print(bvh.root.calculate_positions(np.zeros(3),np.eye(3),0, {}))
        return

if __name__ == "__main__":
    args = get_args()
    hip_transration(args.root_path, args.save_path)
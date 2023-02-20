#!/usr/bin/env python
import numpy as np
import argparse
import math
import pickle
import copy
import json
import os

def rd(x):
    return int(round(x))

class Evaluator(object):
    def __init__(self, filename, aug_map, start, goal, res):
        self.filename = filename
        self.aug_map = aug_map
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.gen_policy()
        self.res = res

    def gen_policy(self):
        self.policy = np.genfromtxt(self.filename)

    def distance_function(self, v):
        return v * 0.5

    def simulate(self, actions):
        pose = np.array(self.start)

        distance = 0
        for v, w in actions:
            pose = self.motion_predict(pose[0], pose[1], pose[2], v, w, self.aug_map)
            distance += self.distance_function(v)
            if pose is None:
                break

        if pose is None:
            # print("Collision!")
            return -1
        elif rd(pose[0]) == self.goal[0] and rd(pose[1]) == self.goal[1]:
            # print('Reach the goal, distance traveled: {}'.format(distance))
            return distance
        else:
            pass

    def evaluate(self):
        actions = self.policy
        distance = self.simulate(actions)
        return 1 / distance

    def motion_predict(self, x, y, theta, v, w, aug_map, dt=0.5, frequency=10):
        """
        This function predicts the next state when transitioning from the current state (x,y,theta) by action (v,w).
        Returns None if the motion results in collision.
        """
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w != 0:
                dx = -v / w * np.sin(theta) + v / w * np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * np.cos(theta + w / frequency)
            else:
                dx = v * np.cos(theta) / frequency
                dy = v * np.sin(theta) / frequency
            x += dx
            y += dy

            x_index = int(math.floor(x / self.res))
            y_index = int(math.floor(y / self.res))
            if (
                x_index >= aug_map.shape[0]
                or x <= 0
                or y_index >= aug_map.shape[1]
                or y <= 0
            ):
                return None
            if aug_map[x_index, y_index] != 0:
                return None
            theta += w / frequency
            if theta >= np.pi * 2:
                theta = theta / (np.pi * 2) * 360
                theta = theta % 360 / 360 * np.pi * 2

        return x, y, theta


class DiscreteEvaluator(Evaluator):
    def gen_policy(self):
        self.policy = np.genfromtxt(self.filename)
        self.policy = self.generate_action_sequence_discrete(self.policy)

    def distance_function(self, v):
        return 1 * v

    def generate_action_sequence_discrete(self, actions):
        controls = []
        for act in actions:
            act = [act[0], act[1] * np.pi / 2]
            controls += [act, act]

        return controls


class MDPEvaluator(Evaluator):
    def gen_policy(self):
        try:
            with open(self.filename, "r") as fin:
                self.policy = json.load(fin)
        except:
            raise ValueError("incorrect policy file")

    def get_action(self, pose):
        position = copy.deepcopy(pose)
        position[0] = round(position[0])
        position[1] = round(position[1])
        position[2] = round(position[2] / (np.pi / 2)) % 4

        position = [str(int(s)) for s in position]
        position = ",".join(position)
        act = copy.deepcopy(self.policy[position])

        if act[0] != 0:
            r = np.random.rand()
            if r < 0.9:
                pass
            elif r < 0.95:
                act = [np.pi / 2, 1]
            else:
                act = [np.pi / 2, -1]

        act[1] = act[1] * np.pi / 2

        return [act, act]

    def simulate(self):
        pose = copy.deepcopy(self.start)
        steps = 0

        while True:
            action = self.get_action(pose)
            for act in action:
                pose = self.motion_predict(
                    pose[0], pose[1], pose[2], act[0], act[1], self.aug_map
                )
                if pose is None:
                    break
                pose = np.array(pose)

            steps += 1

            if pose is None or steps > 100:
                return -1
            elif rd(pose[0]) == self.goal[0] and rd(pose[1]) == self.goal[1]:
                return steps
            else:
                pass

    def evaluate(self):
        records = []
        N = 100
        for i in range(N):
            step = self.simulate()
            spl = float(step > 0) / step
            records.append(spl)

        return np.sum(records) / N


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-g",
        "--goal-file",
        type=str,
        default="../files/goals.json",
        help="JSON file of goals.",
    )
    parser.add_argument(
        "-m",
        "--map-dir",
        type=str,
        default="./logs",
        help="Directory to store aug maps.",
    )
    parser.add_argument(
        "-c",
        "--control-dir",
        type=str,
        default="./Controls",
        help="Directory to store motion plans.",
    )
    args = parser.parse_args()
    with open(args.goal_file, "r") as _f:
        goal_dict = json.load(_f)
    scores = {}
    for map_name in ["map1", "map2", "map3", "map4", "com1building"]:
        goals = goal_dict["%s.png" % map_name]
        map_path = os.path.join(args.map_dir, map_name + ".pkl")
        res = 0.02 if map_name == "com1building" else 0.05
        with open(map_path, "rb") as fin:
            loaded_aug_map = pickle.load(fin)
        for task in ["DSDA", "CSDA", "DSPA"]:
            for goal in goals:
                filename = (
                    task + "_" + map_name + "_" + str(goal[0]) + "_" + str(goal[1])
                )
                plan_path = os.path.join(args.control_dir, filename)
                try:
                    if task == "DSDA":
                        plan_path = plan_path + ".txt"
                        evaluator = DiscreteEvaluator(
                            plan_path, loaded_aug_map, (1, 1, 0), goal, res
                        )
                    elif task == "CSDA":
                        plan_path = plan_path + ".txt"
                        evaluator = Evaluator(
                            plan_path, loaded_aug_map, (1, 1, 0), goal, res
                        )
                    else:
                        plan_path = plan_path + ".json"
                        evaluator = MDPEvaluator(
                            plan_path, loaded_aug_map, (1, 1, 0), goal, res
                        )
                    spl = evaluator.evaluate()
                    print("%s score %s" % (filename, spl))
                    scores[filename] = spl
                except Exception as e:
                    print("Failed to evaluate %s: %s" % (filename, e))
    print(scores)

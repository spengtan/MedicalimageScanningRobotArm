""" main script """
import config
import vrepInterface
import numpy as np
import pyglet
import time

class ArmEnv(object):
    #viewer = None

    dt = .1    # refresh rate
    action_bound = [-1, 1]

    goal = {'x': 0., 'y': 0., 'z': 0., 'r': 0}

    state_dim = 43
    action_dim = 6

    def __init__(self):
        # self.arm_info = np.zeros(
        #     2, dtype=[('l', np.float32), ('r', np.float32)])
        # self.arm_info['l'] = 100        # 2 arms length
        # self.arm_info['r'] = np.pi/6    # 2 angles information

        self.actual_angle = np.zeros(6, dtype=np.float)
        self.on_goal = 0

    # def start(self, i):
    #     if i == 0:
    #         vrepInterface.connect()
    #     vrepInterface.start()

    def get_goal(self):
        pos_goal, ori_goal = vrepInterface.get_novel_pose3d()
        self.goal = {'x': pos_goal[0], 'y': pos_goal[1], 'z': pos_goal[2], 'r': 0.025}
        # print('the true goal is: ', pos_goal)
        return

    def step(self, action):
        done = False
        action = np.clip(action, *self.action_bound)
        self.actual_angle += action*self.dt
        self.actual_angle %= np.pi*2

        joint1_angle = self.actual_angle[0]
        joint2_angle = self.actual_angle[1]
        joint3_angle = self.actual_angle[2]
        joint4_angle = self.actual_angle[3]
        joint5_angle = self.actual_angle[4]
        joint6_angle = self.actual_angle[5]



        vrepInterface.move_joints(joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle)
        time.sleep(1)
        # vrepInterface.stay_sync()

        joint1xyz, joint2xyz, joint3xyz, joint4xyz, joint5xyz, joint6xyz = vrepInterface.get_joints_pose3d()
        fingerxyz, fingerabq = vrepInterface.get_robot_pose3d()

        # vrepInterface.stay_sync()



        # print('joint1xyz is: ', joint1xyz)
        # print('joint2xyz is: ', joint2xyz)
        # print('joint3xyz is: ', joint3xyz)
        # print('joint4xyz is: ', joint4xyz)
        # print('joint5xyz is: ', joint5xyz)
        # print('joint6xyz is: ', joint6xyz)
        # print('fingerxyz is: ', fingerxyz)

        # self.arm_info['r'] += action * self.dt
        # self.arm_info['r'] %= np.pi * 2    # normalize
        #
        # (a1l, a2l) = self.arm_info['l']  # radius, arm length
        # (a1r, a2r) = self.arm_info['r']  # radian, angle
        # a1xy = np.array([200., 200.])    # a1 start (x0, y0)
        # a1xy_ = np.array([np.cos(a1r), np.sin(a1r)]) * a1l + a1xy  # a1 end and a2 start (x1, y1)
        # finger = np.array([np.cos(a1r + a2r), np.sin(a1r + a2r)]) * a2l + a1xy_  # a2 end (x2, y2)





        # normalize features
        # dist1 = [(self.goal['x'] - a1xy_[0]) / 400, (self.goal['y'] - a1xy_[1]) / 400]
        # dist2 = [(self.goal['x'] - finger[0]) / 400, (self.goal['y'] - finger[1]) / 400]
        # r = -np.sqrt(dist2[0]**2+dist2[1]**2)


        dist1 = [(self.goal['x'] - joint1xyz[0]), (self.goal['y'] - joint1xyz[1]), (self.goal['z'] - joint1xyz[2])]
        dist2 = [(self.goal['x'] - joint2xyz[0]), (self.goal['y'] - joint2xyz[1]), (self.goal['z'] - joint2xyz[2])]
        dist3 = [(self.goal['x'] - joint3xyz[0]), (self.goal['y'] - joint3xyz[1]), (self.goal['z'] - joint3xyz[2])]
        dist4 = [(self.goal['x'] - joint4xyz[0]), (self.goal['y'] - joint4xyz[1]), (self.goal['z'] - joint4xyz[2])]
        dist5 = [(self.goal['x'] - joint5xyz[0]), (self.goal['y'] - joint5xyz[1]), (self.goal['z'] - joint5xyz[2])]
        dist6 = [(self.goal['x'] - joint6xyz[0]), (self.goal['y'] - joint6xyz[1]), (self.goal['z'] - joint6xyz[2])]
        dist7 = [(self.goal['x'] - fingerxyz[0]), (self.goal['y'] - fingerxyz[1]), (self.goal['z'] - fingerxyz[2])]

        dist_sum = dist1 + dist2 + dist3 + dist4 + dist5 + dist6 + dist7

        # print('dist is: ', dist_sum, 'shape is: ', len(dist_sum))

        r = -np.sqrt(dist7[0]**2+dist7[1]**2+dist7[2]**2)

        # done and reward
        # if self.goal['x'] - self.goal['l']/2 < finger[0] < self.goal['x'] + self.goal['l']/2:
        #     if self.goal['y'] - self.goal['l']/2 < finger[1] < self.goal['y'] + self.goal['l']/2:
        #         r += 1.
        #         self.on_goal += 1
        #         if self.on_goal > 50:
        #             done = True
        # else:
        #     self.on_goal = 0

        # if fingerxyz[2] - self.goal['z'] > -0.02 and fingerxyz[2] - self.goal['z'] < 0.02:
        #     if np.sqrt((fingerxyz[0]-self.goal['x'])**2+(fingerxyz[1]-self.goal['y'])**2) < self.goal['r']:
        #         r += 1.
        #         self.on_goal += 1
        #         if self.on_goal > 50:
        #             sucess_done = True
        # elif fingerxyz[2] - self.goal['z'] < -0.02:
        #     r += -1
        #     self.on_goal = 0
        #     fail_done = True
        # else:
        #     self.on_goal = 0


        if 0 < fingerxyz[2] - self.goal['z'] < 0.41 and -0.72 < fingerxyz[0] - self.goal['x'] < 0 and 0 < fingerxyz[1] - self.goal['y'] < 1.03:

            if fingerxyz[2] - self.goal['z'] < 0.05 and fingerxyz[0] - self.goal['x'] < 0.05 and fingerxyz[1] - self.goal['y'] < 0.05:

                r += 1.
                self.on_goal += 1
                if self.on_goal > 50:
                    done = True

            else:
                self.on_goal = 0

        else:
            r -= 1.
            self.on_goal = 0





        # state
        # s = np.concatenate((a1xy_/200, finger/200, dist1 + dist2, [1. if self.on_goal else 0.]))
        s = np.concatenate((joint1xyz, joint2xyz, joint3xyz, joint4xyz, joint5xyz, joint6xyz, fingerxyz, dist_sum, [1. if self.on_goal else 0.]))

        return s, r, done

    def reset(self):
        self.goal['x'] = np.random.uniform(0, 0.3)
        self.goal['y'] = np.random.uniform(-0.75, -0.45)
        #self.goal['z'] is constant

        self.actual_angle = 2*np.pi*np.random.rand(6)
        # self.arm_info['r'] = 2 * np.pi * np.random.rand(2)
        # self.on_goal = 0
        # (a1l, a2l) = self.arm_info['l']  # radius, arm length
        # (a1r, a2r) = self.arm_info['r']  # radian, angle
        # a1xy = np.array([200., 200.])  # a1 start (x0, y0)
        # a1xy_ = np.array([np.cos(a1r), np.sin(a1r)]) * a1l + a1xy  # a1 end and a2 start (x1, y1)
        # finger = np.array([np.cos(a1r + a2r), np.sin(a1r + a2r)]) * a2l + a1xy_  # a2 end (x2, y2)
        joint1_angle = self.actual_angle[0]
        joint2_angle = self.actual_angle[1]
        joint3_angle = self.actual_angle[2]
        joint4_angle = self.actual_angle[3]
        joint5_angle = self.actual_angle[4]
        joint6_angle = self.actual_angle[5]

        vrepInterface.move_joints(joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle)
        time.sleep(1)

        # vrepInterface.reset_robot()
        # time.sleep(1)


        # vrepInterface.stay_sync()
        # vrepInterface.stop()
        # vrepInterface.connect()
        # vrepInterface.start()
        joint1xyz, joint2xyz, joint3xyz, joint4xyz, joint5xyz, joint6xyz = vrepInterface.get_joints_pose3d()
        fingerxyz, fingerabq = vrepInterface.get_robot_pose3d()



        # vrepInterface.stay_sync()




        # print('joint1xyz is: ', joint1xyz)
        # print('joint2xyz is: ', joint2xyz)
        # print('joint3xyz is: ', joint3xyz)
        # print('joint4xyz is: ', joint4xyz)
        # print('joint5xyz is: ', joint5xyz)
        # print('joint6xyz is: ', joint6xyz)
        # print('fingerxyz is: ', fingerxyz)
        #
        # print('goalxyz is: ', self.goal['x'], self.goal['y'], self.goal['z'])

        # normalize features
        #dist1 = [(self.goal['x'] - a1xy_[0])/400, (self.goal['y'] - a1xy_[1])/400]
        #dist2 = [(self.goal['x'] - finger[0])/400, (self.goal['y'] - finger[1])/400]
        dist1 = [(self.goal['x'] - joint1xyz[0]), (self.goal['y'] - joint1xyz[1]), (self.goal['z'] - joint1xyz[2])]
        dist2 = [(self.goal['x'] - joint2xyz[0]), (self.goal['y'] - joint2xyz[1]), (self.goal['z'] - joint2xyz[2])]
        dist3 = [(self.goal['x'] - joint3xyz[0]), (self.goal['y'] - joint3xyz[1]), (self.goal['z'] - joint3xyz[2])]
        dist4 = [(self.goal['x'] - joint4xyz[0]), (self.goal['y'] - joint4xyz[1]), (self.goal['z'] - joint4xyz[2])]
        dist5 = [(self.goal['x'] - joint5xyz[0]), (self.goal['y'] - joint5xyz[1]), (self.goal['z'] - joint5xyz[2])]
        dist6 = [(self.goal['x'] - joint6xyz[0]), (self.goal['y'] - joint6xyz[1]), (self.goal['z'] - joint6xyz[2])]
        dist7 = [(self.goal['x'] - fingerxyz[0]), (self.goal['y'] - fingerxyz[1]), (self.goal['z'] - fingerxyz[2])]

        dist_sum = dist1+dist2+dist3+dist4+dist5+dist6+dist7

        # state
        #s = np.concatenate((a1xy_/200, finger/200, dist1 + dist2, [1. if self.on_goal else 0.]))
        s = np.concatenate((joint1xyz, joint2xyz, joint3xyz, joint4xyz, joint5xyz, joint6xyz, fingerxyz, dist_sum, [1. if self.on_goal else 0.]))

        return s

    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(self.arm_info, self.goal)
        self.viewer.render()

    def sample_action(self):
        return np.random.rand(6)-0.5    # two radians

    def startPause():
        vrepInterface.pause()


    def endPause():
        vrepInterface.start()




class Viewer(pyglet.window.Window):
    bar_thc = 5

    def __init__(self, arm_info, goal):
        # vsync=False to not use the monitor FPS, we can speed up training
        super(Viewer, self).__init__(width=400, height=400, resizable=False, caption='Arm', vsync=False)
        pyglet.gl.glClearColor(1, 1, 1, 1)
        self.arm_info = arm_info
        self.goal_info = goal
        self.center_coord = np.array([200, 200])

        self.batch = pyglet.graphics.Batch()    # display whole batch at once
        self.goal = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,    # 4 corners
            ('v2f', [goal['x'] - goal['l'] / 2, goal['y'] - goal['l'] / 2,                # location
                     goal['x'] - goal['l'] / 2, goal['y'] + goal['l'] / 2,
                     goal['x'] + goal['l'] / 2, goal['y'] + goal['l'] / 2,
                     goal['x'] + goal['l'] / 2, goal['y'] - goal['l'] / 2]),
            ('c3B', (86, 109, 249) * 4))    # color
        self.arm1 = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', [250, 250,                # location
                     250, 300,
                     260, 300,
                     260, 250]),
            ('c3B', (249, 86, 86) * 4,))    # color
        self.arm2 = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', [100, 150,              # location
                     100, 160,
                     200, 160,
                     200, 150]), ('c3B', (249, 86, 86) * 4,))

    def render(self):
        self._update_arm()
        self.switch_to()
        self.dispatch_events()
        self.dispatch_event('on_draw')
        self.flip()

    def on_draw(self):
        self.clear()
        self.batch.draw()

    def _update_arm(self):
        # update goal
        self.goal.vertices = (
            self.goal_info['x'] - self.goal_info['l']/2, self.goal_info['y'] - self.goal_info['l']/2,
            self.goal_info['x'] + self.goal_info['l']/2, self.goal_info['y'] - self.goal_info['l']/2,
            self.goal_info['x'] + self.goal_info['l']/2, self.goal_info['y'] + self.goal_info['l']/2,
            self.goal_info['x'] - self.goal_info['l']/2, self.goal_info['y'] + self.goal_info['l']/2)

        # update arm
        (a1l, a2l) = self.arm_info['l']     # radius, arm length
        (a1r, a2r) = self.arm_info['r']     # radian, angle
        a1xy = self.center_coord            # a1 start (x0, y0)
        a1xy_ = np.array([np.cos(a1r), np.sin(a1r)]) * a1l + a1xy   # a1 end and a2 start (x1, y1)
        a2xy_ = np.array([np.cos(a1r+a2r), np.sin(a1r+a2r)]) * a2l + a1xy_  # a2 end (x2, y2)

        a1tr, a2tr = np.pi / 2 - self.arm_info['r'][0], np.pi / 2 - self.arm_info['r'].sum()
        xy01 = a1xy + np.array([-np.cos(a1tr), np.sin(a1tr)]) * self.bar_thc
        xy02 = a1xy + np.array([np.cos(a1tr), -np.sin(a1tr)]) * self.bar_thc
        xy11 = a1xy_ + np.array([np.cos(a1tr), -np.sin(a1tr)]) * self.bar_thc
        xy12 = a1xy_ + np.array([-np.cos(a1tr), np.sin(a1tr)]) * self.bar_thc

        xy11_ = a1xy_ + np.array([np.cos(a2tr), -np.sin(a2tr)]) * self.bar_thc
        xy12_ = a1xy_ + np.array([-np.cos(a2tr), np.sin(a2tr)]) * self.bar_thc
        xy21 = a2xy_ + np.array([-np.cos(a2tr), np.sin(a2tr)]) * self.bar_thc
        xy22 = a2xy_ + np.array([np.cos(a2tr), -np.sin(a2tr)]) * self.bar_thc

        self.arm1.vertices = np.concatenate((xy01, xy02, xy11, xy12))
        self.arm2.vertices = np.concatenate((xy11_, xy12_, xy21, xy22))

    # convert the mouse coordinate to goal's coordinate
    def on_mouse_motion(self, x, y, dx, dy):
        self.goal_info['x'] = x
        self.goal_info['y'] = y


if __name__ == '__main__':
    env = ArmEnv()
    while True:
        env.render()
        env.step(env.sample_action())
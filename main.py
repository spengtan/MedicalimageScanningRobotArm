"""
Make it more robust.
Stop episode once the finger stop at the final position for 50 steps.
Feature & reward engineering.
"""
#from final.env import ArmEnv
#from final.rl import DDPG
from env import ArmEnv
from rl import DDPG
import vrepInterface

#MAX_EPISODES = 900
MAX_EPISODES = 900
#MAX_EP_STEPS = 200
MAX_EP_STEPS = 100
ON_TRAIN = True

# set env
env = ArmEnv()
s_dim = env.state_dim
a_dim = env.action_dim
a_bound = env.action_bound

# set RL method (continuous)
rl = DDPG(a_dim, s_dim, a_bound)

steps = []
def train():

    vrepInterface.connect()
    vrepInterface.start()

    # start training
    for i in range(MAX_EPISODES):
        env.get_goal()
        s = env.reset()
        # print('the start s is: ',s)
        ep_r = 0.
        for j in range(MAX_EP_STEPS):
            # env.render()

            a = rl.choose_action(s)
            # print('STEP ', j, ' a is: ', a)


            s_, r, done = env.step(a)
            # print('STEP ', j, 's is :', s_)
            # print('STEP ', j, 'r is :', r)

            rl.store_transition(s, a, r, s_)

            ep_r += r
            if rl.memory_full:
                # start to learn once has fulfilled the memory
                rl.learn()
                # print('start to learn')

            s = s_
            if done or j == MAX_EP_STEPS-1:
                print('Ep: %i | %s | ep_r: %.1f | step: %i' % (i, '---' if not done else 'done', ep_r, j))
                break
    rl.save()
    vrepInterface.disconnect()


def eval():
    rl.restore()
    vrepInterface.connect()
    vrepInterface.start()
    # env.render()
    # env.viewer.set_vsync(True)
    env.get_goal()
    s = env.reset()
    while True:
        # env.render()
        a = rl.choose_action(s)
        s, r, done = env.step(a)


if ON_TRAIN:
    train()
else:
    eval()




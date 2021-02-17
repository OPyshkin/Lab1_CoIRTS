"""
Bug Planning
author: Sarim Mehdi(muhammadsarim.mehdi@studio.unibo.it)
Source: https://sites.google.com/site/ece452bugalgorithms/
"""

import numpy as np
import matplotlib.pyplot as plt
import time, math

show_animation = True



class BugPlanner:
    def __init__(self, start_x, start_y, goal_x, goal_y, obs_x, obs_y, phi):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.obs_x = obs_x #Black dots an obstacle consists of (x-coords)
        self.obs_y = obs_y #Black dots an obstacle consists of (y-coords)
        self.r_x = [start_x] #Robot x
        self.r_y = [start_y] #Robot y
        self.out_x = [] #Blue dots around an obstacle x-coords
        self.out_y = [] #Blue dots around an obstacle y-coords
        self.phi = phi
        for o_x, o_y in zip(obs_x, obs_y):
            for add_x, add_y in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                    [1, 1, 1, 0, -1, -1, -1, 0]):
                cand_x, cand_y = o_x+add_x, o_y+add_y
                valid_point = True
                for _x, _y in zip(obs_x, obs_y):
                    if cand_x == _x and cand_y == _y:
                        valid_point = False
                        break
                if valid_point:
                    self.out_x.append(cand_x), self.out_y.append(cand_y)

    def mov_normal(self):      ### M-line motion
        return self.r_x[-1] + math.cos(self.phi), \
               self.r_y[-1] + math.sin(self.phi)
        
                   
    # TODO: FIX MOTION AROUND AN OBSTACLE
    def mov_to_next_obs(self, visited_x, visited_y):
        for add_x, add_y in zip([1, 0, -1, 0], [0, 1, 0, -1]):
            c_x, c_y = math.ceil(self.r_x[-1]) + add_x, \
                       math.ceil(self.r_y[-1]) + add_y
            for _x, _y in zip(self.out_x, self.out_y):
                use_pt = True
                if c_x == _x and c_y == _y:
                    for v_x, v_y in zip(visited_x, visited_y):
                        if c_x == v_x and c_y == v_y:
                            use_pt = False
                            break
                    if use_pt:
                        return c_x, c_y, False
                if not use_pt:
                    break
        return math.ceil(self.r_x[-1]), math.ceil(self.r_y[-1]), True

    def bug0(self):
        """
        Greedy algorithm where you move towards goal
        until you hit an obstacle. Then you go around it
        (pick an arbitrary direction), until it is possible
        for you to start moving towards goal in a greedy manner again
        """
        #start_time = time.time()
        # TODO: INSERT CONSTANT ROBOT MOTION MODEL INTO DESCRETE ENVIRONMENT MODEL
        # TODO: REANALIZE FUCTIONS OF cand AND r_ 
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf ####Like...candidate???
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")
            plt.grid(True)
            plt.title('BUG 0')

        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if math.ceil(self.r_x[-1]) == x_ob and math.ceil(self.r_y[-1]) == y_ob:
                print("AAAAAA")
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            if cand_x == self.goal_x and \
                    cand_y == self.goal_y:
                break
            if mov_dir == 'normal':
                real_x, real_y = self.mov_normal()
                cand_x = math.ceil(real_x)
                cand_y = math.ceil(real_y)
            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(real_x), self.r_y.append(real_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        mov_dir = 'obs'
                        print("OOOOOOOOOOOOOOO")
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(real_x), self.r_y.append(real_y)
            elif mov_dir == 'obs':
                can_go_normal = True
                for x_ob, y_ob in zip(self.obs_x, self.obs_y):
                    if math.ceil(self.mov_normal()[0]) == x_ob and \
                            math.ceil(self.mov_normal()[1]) == y_ob:
                        can_go_normal = False
                        break
                if can_go_normal:
                    mov_dir = 'normal'
                else:
                    self.r_x.append(real_x), self.r_y.append(real_y)
                    visited_x.append(cand_x), visited_y.append(cand_y)
            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)
        if show_animation:
            plt.show()
	
        ###print(time.time() - start_time)

    def bug1(self):
        """
        Move towards goal in a greedy manner.
        When you hit an obstacle, you go around it and
        back to where you hit the obstacle initially.
        Then, you go to the point on the obstacle that is
        closest to your goal and you start moving towards
        goal in a greedy manner from that new point.
        """
        start_time = time.time()
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        exit_x, exit_y = -np.inf, -np.inf
        dist = np.inf
        back_to_start = False
        second_round = False
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")
            plt.grid(True)
            plt.title('BUG 1')

        for xob, yob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == xob and self.r_y[-1] == yob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            if self.r_x[-1] == self.goal_x and \
                    self.r_y[-1] == self.goal_y:
                break
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()
            if mov_dir == 'obs':
                cand_x, cand_y, back_to_start = \
                    self.mov_to_next_obs(visited_x, visited_y)
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        mov_dir = 'obs'
                        dist = np.inf
                        back_to_start = False
                        second_round = False
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
            elif mov_dir == 'obs':
                d = np.linalg.norm(np.array([cand_x, cand_y] -
                                            np.array([self.goal_x,
                                                      self.goal_y])))
                if d < dist and not second_round:
                    exit_x, exit_y = cand_x, cand_y
                    dist = d
                if back_to_start and not second_round:
                    second_round = True
                    del self.r_x[-len(visited_x):]
                    del self.r_y[-len(visited_y):]
                    visited_x[:], visited_y[:] = [], []
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)
                if cand_x == exit_x and \
                        cand_y == exit_y and \
                        second_round:
                    mov_dir = 'normal'
            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)
        if show_animation:
            plt.show()
        
        print(time.time() - start_time)
"""""""""
— Здорово, Федь!
— Здорово!
— Как оно?
— Да чё!
— Давай руку. У, бля, тоже спортом позаниматься что ль, блядь?
— Давай!
— Ты давно здесь?
— Да не
— О, блядь. Оттак, оп! Ху-ху-ху. Поотжиматься, оппа, на две выход. Оп, вот так, нахуй. Х, ха. Нинора... Блядь, тепло сёдня, блядь
— Ну
— На улице, блядь. Ебать в рот. Ха. Ох. Щас штангу хочу потягать, ёпта. Ух ты, бляха-муха, тяжёлая-то какая. Оппа. Кхм-Кхм. Х. Федь, поможешь? Поднакинь
— Давай
— Оппана. О! Хоть спортом позаниматься. Ух ты, бля, кх. Ху. Хоба. Ох ты, бля. АА! А-А. Ах. О! *пердёж* А-КВАА ФЕДЯ! *продолжительный пердёж* ФЕДЯ, ШПИНГАЛЕТЫ ЛОПНУЛИ! АХ! ОООХ. БЛЯДЬ! Федь, обосрался, шпингалеты лопнули, ОО, руки вспот.. АА, ОО, О, О, ОО, О, О
— Ух ёбаный в рот, завоняло-то блядь, чё здесь воняет-то, блядь?
— Федь, у меня... О... О...
— Фу, блядь
— КХ, ООО, О...
— Всё
"""""""""
    def bug2(self):
        """
        Move towards goal in a greedy manner.
        When you hit an obstacle, you go around it and
        keep track of your distance from the goal.
        If the distance from your goal was decreasing before
        and now it starts increasing, that means the current
        point is probably the closest point to the
        goal (this may or may not be true because the algorithm
        doesn't explore the entire boundary around the obstacle).
        So, you depart from this point and continue towards the
        goal in a greedy manner
        """
        start_time = time.time()
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")

        straight_x, straight_y = [self.r_x[-1]], [self.r_y[-1]]
        hit_x, hit_y = [], []
        while True:
            if straight_x[-1] == self.goal_x and \
                    straight_y[-1] == self.goal_y:
                break
            c_x = straight_x[-1] + np.sign(self.goal_x - straight_x[-1])
            c_y = straight_y[-1] + np.sign(self.goal_y - straight_y[-1])
            for x_ob, y_ob in zip(self.out_x, self.out_y):
                if c_x == x_ob and c_y == y_ob:
                    hit_x.append(c_x), hit_y.append(c_y)
                    break
            straight_x.append(c_x), straight_y.append(c_y)
        if show_animation:
            plt.plot(straight_x, straight_y, ",")
            plt.plot(hit_x, hit_y, "d")
            plt.grid(True)
            plt.title('BUG 2')

        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            if self.r_x[-1] == self.goal_x \
                    and self.r_y[-1] == self.goal_y:
                break
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()
            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        del hit_x[0]
                        del hit_y[0]
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
            elif mov_dir == 'obs':
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)
                for i_x, i_y in zip(range(len(hit_x)), range(len(hit_y))):
                    if cand_x == hit_x[i_x] and cand_y == hit_y[i_y]:
                        del hit_x[i_x]
                        del hit_y[i_y]
                        mov_dir = 'normal'
                        break
            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)
        if show_animation:
            plt.show()

        print(time.time() - start_time)

def main(bug_0, bug_1, bug_2):
    # set obstacle positions
    o_x, o_y = [], []

    s_x = 35.0
    s_y = 0.0
    g_x = 35.0
    g_y = 70.0
    if (s_x == g_x):
        phi = (math.pi / 2) * np.sign(g_y - s_y)
    else:
        phi = math.atan((g_y - s_y) / (g_x - s_x))

    
    for i in range(10, 40):
        for j in range(10, 15):
            o_x.append(i)
            o_y.append(j)

    for i in range(50, 60):
        for j in range(10, 15):
            o_x.append(i)
            o_y.append(j)

    for i in range(10, 15):
        for j in range(15, 60):
            o_x.append(i)
            o_y.append(j)

    for i in range(55, 60):
        for j in range(15, 60):
            o_x.append(i)
            o_y.append(j)

    for i in range(10, 60):
        for j in range(55, 60):
            o_x.append(i)
            o_y.append(j)

    for i in range(30, 55):
        for j in range(25, 30):
            o_x.append(i)
            o_y.append(j)

    for i in range(10, 40):
        for j in range(40, 45):
            o_x.append(i)
            o_y.append(j)
    



    if bug_0:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y, phi)
        my_Bug.bug0()
    if bug_1:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
        my_Bug.bug1()
    if bug_2:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
        my_Bug.bug2()


if __name__ == '__main__':
    main(bug_0=True, bug_1=False, bug_2=False)

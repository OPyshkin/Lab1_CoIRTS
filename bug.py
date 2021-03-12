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

    def my_round(self, num):
        if num % 1 >= 0.5:
            return math.ceil(num)
        else:
            return num // 1

    def mov_normal(self):     ### M-line motion
        if self.r_x[0] == self.goal_x:
            return self.r_x[0], \
                   self.r_y[-1] + math.sin(self.phi)
        return self.r_x[-1] + math.cos(self.phi), \
               self.r_y[-1] + math.sin(self.phi)
                   
    def mov_to_next_obs(self, visited_x, visited_y):
        
        for add_x, add_y in zip([1, 0, -1, 0], [0, 1, 0, -1]):
            c_x, c_y = self.my_round(self.r_x[-1]) + add_x, \
                       self.my_round(self.r_y[-1]) + add_y
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
        return visited_x[0], visited_y[0], True
        

    def bug0(self):
        """
        Greedy algorithm where you move towards goal
        until you hit an obstacle. Then you go around it
        (pick an arbitrary direction), until it is possible
        for you to start moving towards goal in a greedy manner again
        """
        start_time = time.time() #FIRST TIME CHECKPOINT

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
            if self.my_round(self.r_x[-1]) == x_ob and self.my_round(self.r_y[-1]) == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            #goal achievement condition
            if cand_x == self.goal_x and \
                    cand_y == self.goal_y:
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                break
            if math.fabs(self.r_x[-1]) > 100 or math.fabs(self.r_y[-1]) > 100: #test condition (to be cut)
                break
            if mov_dir == 'normal': #normal motion step
                real_x, real_y = self.mov_normal()
                cand_x = self.my_round(real_x)
                cand_y = self.my_round(real_y)
            if mov_dir == 'obs': #obstacle motion step
                cand_x, cand_y, was_here = self.mov_to_next_obs(visited_x, visited_y)
                if was_here: #no suitable way found condition
                    break
            if mov_dir == 'normal': #normal motion until obstacle is found
                found_boundary = False
                
                for x_ob, y_ob in zip(self.out_x, self.out_y): #obstacle approach check 
                    if cand_x == x_ob and cand_y == y_ob:
                        preobs_x = cand_x
                        preobs_y = cand_y
                
                for x_ob, y_ob in zip(self.obs_x, self.obs_y): #obstacle detection
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(preobs_x), self.r_y.append(preobs_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(preobs_x), visited_y.append(preobs_y)
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(real_x), self.r_y.append(real_y)
            elif mov_dir == 'obs':
                can_go_normal = True
                path_clear = True 
                if (cand_x == self.goal_x): #new m-line angle calculation
                    self.phi = (math.pi / 2) * np.sign(self.goal_y - cand_y)
                else:
                    self.phi = math.atan((self.goal_y - cand_y) / (self.goal_x - cand_x))
                if self.goal_x < cand_x:
                    self.phi += math.pi

                a = min(self.goal_x, cand_x)
                b = max(self.goal_x, cand_x)

                x_m = []
                y_m = []
                
                d = 0
                if a == b:
                    a = min(self.goal_y, cand_y)
                    b = max(self.goal_y, cand_y)
                    while a + d <= b:
                        x_m.append(cand_x)
                        y_m.append(a + d)
                        d += 1
                else:
                    while a + d <= b:
                        x_m.append(a + d)
                        d += 0.076923
                    for x in list(x_m): 
                        y = (math.tan(self.phi) * (x - cand_x) + cand_y)
                        y_m.append(y)
                
                for x, y in zip(self.obs_x, self.obs_y):
                    for x_, y_ in zip(x_m, y_m):
                        if self.my_round(x_) == x and self.my_round(y_) == y:
                            path_clear = False
                            break

                c_real_x, c_real_y = self.mov_normal()
                for x_ob, y_ob in zip(self.obs_x, self.obs_y):
                    if self.my_round(c_real_x + np.sign(math.cos(self.phi)) / 2) == x_ob and \
                       self.my_round(c_real_y + np.sign(math.sin(self.phi)) / 2) == y_ob:
                        can_go_normal = False
                        break

                if can_go_normal and path_clear:
                    mov_dir = 'normal'
                else:
                    visited_x.append(cand_x), visited_y.append(cand_y)
                self.r_x.append(cand_x), self.r_y.append(cand_y)
            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                #plt.pause(0.1)
        print(time.time() - start_time) #SECOND TIME CHECKPOINT
        if show_animation:
            plt.show()

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
        go_back = False
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")
            plt.grid(True)
            plt.title('BUG 1')

        for x_ob, y_ob in zip(self.out_x, self.out_y): 
            if self.my_round(self.r_x[-1]) == x_ob and self.my_round(self.r_y[-1]) == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            #goal achievement condition
            if cand_x == self.goal_x and \
                    cand_y == self.goal_y:
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                break
            if math.fabs(self.r_x[-1]) > 100 or math.fabs(self.r_y[-1]) > 100: #test condition (to be cut)
                break

            if mov_dir == 'normal': #normal motion step
                real_x, real_y = self.mov_normal()
                cand_x = self.my_round(real_x)
                cand_y = self.my_round(real_y)


            if mov_dir == 'obs': #obstacle motion step
                cand_x, cand_y, back_to_start = self.mov_to_next_obs(visited_x, visited_y)


            if mov_dir == 'normal': #normal motion until obstacle is found
                found_boundary = False
                
                for x_ob, y_ob in zip(self.out_x, self.out_y): #obstacle approach check 
                    if cand_x == x_ob and cand_y == y_ob:
                        preobs_x = cand_x
                        preobs_y = cand_y
                
                for x_ob, y_ob in zip(self.obs_x, self.obs_y): #obstacle detection
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(preobs_x), self.r_y.append(preobs_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(preobs_x), visited_y.append(preobs_y)
                        mov_dir = 'obs'
                        dist = np.inf #
                        back_to_start = False #
                        go_back = False #
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(real_x), self.r_y.append(real_y)

            elif mov_dir == 'obs':
                d = np.linalg.norm(np.array([cand_x, cand_y] -
                                            np.array([self.goal_x,
                                                      self.goal_y])))
                if d < dist and not go_back:
                    exit_x, exit_y = cand_x, cand_y
                    dist = d
                if back_to_start and not go_back:
                    go_back = True 
                    self.r_x, self.r_y = [], []
                    del visited_x[2:]
                    del visited_x[0]
                    del visited_y[2:]
                    del visited_y[0]

                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)
                if cand_x == exit_x and \
                        cand_y == exit_y and \
                        go_back:
                    mov_dir = 'normal'

                    if (cand_x == self.goal_x): #new m-line angle calculation
                        self.phi = (math.pi / 2) * np.sign(self.goal_y - cand_y)
                    else:
                        self.phi = math.atan((self.goal_y - cand_y) / (self.goal_x - cand_x))
                    if self.goal_x < cand_x:
                        self.phi += math.pi

            if show_animation:
                if go_back:
                    
                    plt.plot(self.r_x, self.r_y, "-b")
                else:
                    plt.plot(self.r_x, self.r_y, "-r")
                #plt.pause(0.1)
        print(time.time() - start_time)
        if show_animation:
            plt.show()

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
        exit_x, exit_y = -np.inf, -np.inf
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
            if self.my_round(straight_x[-1]) == self.goal_x and \
                    self.my_round(straight_y[-1]) == self.goal_y:
                break
            """
            c_x = straight_x[-1] + np.sign(self.goal_x - straight_x[-1])
            c_y = straight_y[-1] + np.sign(self.goal_y - straight_y[-1])
            """
            c_real_x = straight_x[-1] + math.cos(self.phi)
            c_real_y = straight_y[-1] + math.sin(self.phi)
            c_x = self.my_round(c_real_x)
            c_y = self.my_round(c_real_y)

            for x_ob, y_ob in zip(self.out_x, self.out_y):
                if c_x == x_ob and c_y == y_ob:
                    hit_x.append(c_real_x), hit_y.append(c_real_y)
                    break
            straight_x.append(c_real_x), straight_y.append(c_real_y)

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
            #goal achievement condition
            if cand_x == self.goal_x and \
                    cand_y == self.goal_y:
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                break
            if math.fabs(self.r_x[-1]) > 100 or math.fabs(self.r_y[-1]) > 100: #test condition (to be cut)
                break

            if mov_dir == 'normal': #normal motion step
                real_x, real_y = self.mov_normal()
                cand_x = self.my_round(real_x)
                cand_y = self.my_round(real_y)
            
            if mov_dir == 'obs': #obstacle motion step
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)

            if mov_dir == 'normal': #normal motion until obstacle is found
                found_boundary = False
                
                for x_ob, y_ob in zip(self.out_x, self.out_y): #obstacle approach check 
                    if cand_x == x_ob and cand_y == y_ob:
                        preobs_x = cand_x
                        preobs_y = cand_y
                
                for x_ob, y_ob in zip(self.obs_x, self.obs_y): #obstacle detection
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(preobs_x), self.r_y.append(preobs_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(preobs_x), visited_y.append(preobs_y)
                        mov_dir = 'obs'
                        del hit_x[0]
                        del hit_y[0]
                        found_boundary = True
                        
                if not found_boundary:
                    self.r_x.append(real_x), self.r_y.append(real_y)

            elif mov_dir == 'obs':
                go_m = True
                m_crossed = False #m_line crossing detection
                for x_m, y_m in zip(hit_x, hit_y):
                    if self.my_round(x_m) == cand_x and self.my_round(y_m) == cand_y:
                        exit_x = x_m
                        exit_y = y_m
                        m_crossed = True
                        break

                c_real_x, c_real_y = self.mov_normal()
                for x_ob, y_ob in zip(self.obs_x, self.obs_y):
                    if self.my_round(c_real_x) == x_ob and \
                       self.my_round(c_real_y) == y_ob:
                        go_m = False
                        break
                
                if m_crossed and go_m:
                    mov_dir = 'normal'
                    self.r_x.append(exit_x), self.r_y.append(exit_y)
                else:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
                    visited_x.append(cand_x), visited_y.append(cand_y)

            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                #plt.pause(0.1)
        print(time.time() - start_time)
        if show_animation:
            plt.show()

def main(bug_0, bug_1, bug_2):

    def my_round(num):
        if a % 1 >= 0.5:
            return math.ceil(a)
        else:
            return a // 1

    # set obstacle positions
    o_x, o_y = [], []

    s_x = 0.0
    s_y = 0.0
    g_x = 70.0
    g_y = 70.0
    if (s_x == g_x):
        phi = (math.pi / 2) * np.sign(g_y - s_y)
    else:
        phi = math.atan((g_y - s_y) / (g_x - s_x))
    if g_x < s_x:
        phi += math.pi
    """
    for i in range(24, 30):
        for j in range(15, 25):
            o_x.append(i)
            o_y.append(j)
    

    for i in range(20, 40):
        for j in range(30, 35):
            o_x.append(i)
            o_y.append(j)

    for i in range(35, 40):
        for j in range(15, 30):
            o_x.append(i)
            o_y.append(j)

    """
    for i in range(10, 40):
        for j in range(10, 15):
            o_x.append(i)
            o_y.append(j)

    for i in range(50, 60):
        for j in range(10, 15):
            o_x.append(i)
            o_y.append(j)

    for i in range(40, 45):
        for j in range(34, 45):
            o_x.append(i)
            o_y.append(j)

    for i in range(10, 15):
        for j in range(15, 21):
            o_x.append(i)
            o_y.append(j)

    for i in range(10, 15):
        for j in range(26, 60):
            o_x.append(i)
            o_y.append(j)

    for i in range(55, 60):
        for j in range(15, 44):
            o_x.append(i)
            o_y.append(j)

    for i in range(55, 60):
        for j in range(49, 60):
            o_x.append(i)
            o_y.append(j)

    for i in range(10, 20):
        for j in range(55, 60):
            o_x.append(i)
            o_y.append(j)

    for i in range(30, 60):
        for j in range(55, 60):
            o_x.append(i)
            o_y.append(j)

    for i in range(30, 55):
        for j in range(25, 30):
            o_x.append(i)
            o_y.append(j)

    for i in range(25, 30):
        for j in range(25, 36):
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
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y, phi)
        my_Bug.bug1()
    if bug_2:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y, phi)
        my_Bug.bug2()


if __name__ == '__main__':
    main(bug_0=True, bug_1=True, bug_2=True)

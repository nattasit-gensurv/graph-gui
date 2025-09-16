import numpy as np
import math
import networkx as nx
import matplotlib.pyplot as plt
from .default_path_finder import plan_dubins_path
from .dubins_path_planning import dubins_path,hybrid_astar,rrt


_PATH_TYPES = ["straight", "arc", "easydubins", "dubins", "hybrid_astar", "rrt"]

class Obstacle:
    '''
    obstacle = list of tuples representing the (x, y) coordinates of the obstacle on map
    '''
    def __init__(self):
        self.obstacle = []

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
    def clear_obstacles(self):
        self.obstacles = []

class Robot:
    '''
    max_speed : m/s
    max_steer : degs
    '''
    def __init__(self, 
                 name: str=None,
                 type: str=None,
                 length: float=None , 
                 max_speed: float=None, 
                 max_steer: float=None,
                 current_pose: tuple=None,
                 turning_radius: float=None):
        try:
            self.length = length
            self.max_speed = max_speed
            self.max_steer = np.deg2rad(max_steer)
            assert len(current_pose) == 3, "current_pose should be (x, y, theta )"
            self.current_pose = current_pose
            self.status_code = ""
            self.path = []  
        except:
            pass

    def set_pose(self, pose: tuple):
        self.current_pose = pose

    def clear_pose(self):
        self.current_pose = (None, None, None)

    def update_pose(self, pose: tuple):
        self.current_pose = pose

    def update_path(self, path: list):
        self.path = path
    
    def clear_path(self):
        self.path = []


class TagNode:
    '''
    tags = list of {id: str , pose: float(x, y, theta), role: int}\n
    role's enum 0: depot, 1: station, 2: viapoints
    '''
    def __init__(self, id: str, pose: tuple, role: int):
        assert role in [0, 1, 2], "Invalid role"
        assert len(pose) == 3, "Pose should be (x, y, theta)"

        self.id = id
        self.pose = pose
        self.role = role
        self.edges = []  # int: other TagNode ids)
    def edit(self, id: str=None, pose: tuple=None, role: int=None):
        if id is not None:
            self.id = id
        if pose is not None:
            self.pose = pose
        if role is not None:
            self.role = role
    
    def add_edge(self, edge:int):
        self.edges.append(edge)

    def offset(self, dx: float =0.0, dy: float =0.0, dtheta: float =0.0):
        x, y, theta = self.pose
        self.pose = (x + dx, y + dy, theta + dtheta)

class TagMap:
    def __init__(self, nodes: TagNode=None):
        self.tags = {}
        if nodes:
            self.add_nodes(nodes)

    def add_nodes(self, nodes):
        for node in nodes:
            assert isinstance(node, TagNode), "Must be TagNode object"
            self.tags[node.id] = node

    def add_node(self, node: TagNode):
        assert isinstance(node, TagNode), "Must be TagNode object"
        self.tags[node.id] = node

    def get_tag(self, id: str):
        return self.tags.get(id)

    def remove_tag(self, id: str):
        if id in self.tags:
            del self.tags[id]

    def clear_tags(self):
        self.tags = {}

    def plot(self):
        if not self.tags:
            print("No tags to plot.")
            return

        role_colors = {0: "red", 1: "blue", 2: "green"}
        role_labels = {0: "Depot", 1: "Station", 2: "Viapoint"}

        plt.figure(figsize=(7, 7))
        ax = plt.gca()

        for tag in self.tags.values():
            x, y, theta = tag.pose
            dx = math.cos(theta) * 0.5
            dy = math.sin(theta) * 0.5
            plt.scatter(x, y, color=role_colors[tag.role], label=role_labels[tag.role])
            plt.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2,
                      fc=role_colors[tag.role], ec=role_colors[tag.role])
            plt.text(x + 0.1, y + 0.1, f"{tag.id}", fontsize=9)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("TagMap Nodes")
        plt.grid(True)
        plt.axis("equal")
        plt.show()

class Utils:
    def __init__(self):
        self.debug = False

    def plot_tag(self, tagmap: TagMap):
        for tag in tagmap.tags.values():
            x, y, theta = tag.pose
            plt.scatter(x, y, label=tag.id)

    def plot_path_with_arrows(self,path, color='r', arrow_step=5,label='Path'):
        px, py = zip(*path)
        plt.plot(px, py, color+'-', label=label)
        # Draw arrows along the path
        for i in range(0, len(path)-1, arrow_step):
            dx = px[i+1] - px[i]
            dy = py[i+1] - py[i]
            plt.arrow(px[i], py[i], dx, dy, head_width=0.03, head_length=0.02, fc=color, ec=color)
    def show(self):
        plt.axis("equal"); plt.legend(); plt.grid(True)
        plt.show()
        
    def path_equation(self,path, path_type="auto"):
        def fit_circle(path):
            x1, y1 = path[0]
            x2, y2 = path[len(path)//2]
            x3, y3 = path[-1]
            A = np.array([[x2-x1, y2-y1],
                        [x3-x2, y3-y2]])
            B = np.array([[(x2**2 - x1**2 + y2**2 - y1**2)/2],
                        [(x3**2 - x2**2 + y3**2 - y2**2)/2]])
            try:
                cx, cy = np.linalg.lstsq(A, B, rcond=None)[0].flatten()
            except:
                cx, cy = np.nan, np.nan
            R = np.sqrt((x1 - cx)**2 + (y1 - cy)**2)
            return cx, cy, R
        def fit_line(path):
            x1, y1 = path[0]
            x2, y2 = path[-1]
            m = (y2 - y1) / (x2 - x1)
            b = y1 - m * x1
            return m,b
        if path_type == "straight":
            return fit_line(path)
        elif path_type == "arc":
            return fit_circle(path)
        else:  # auto
            # check if nearly straight
            x0, y0 = path[0]
            xn, yn = path[-1]
            d_total = np.hypot(xn-x0, yn-y0)
            d_path = sum(np.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) 
                        for i in range(len(path)-1))
            if abs(d_path - d_total) < 1e-3:
                return fit_line(path)
            else:
                return fit_circle(path)
    def plot_parametric_with_arrows(self,f, color='r', arrow_step=0.05,label="None"):
        t_vals = np.arange(0, 1+arrow_step, arrow_step)
        xy_vals = [f(t) for t in t_vals]
        px, py = zip(*xy_vals)
        plt.plot(px, py, color+'-',label=label)
        # arrows
        for i in range(0, len(px)-1, max(1,int(len(px)*arrow_step))):
            dx = px[i+1]-px[i]
            dy = py[i+1]-py[i]
            plt.arrow(px[i], py[i], dx, dy, head_width=0.01, head_length=0.02, fc=color, ec=color)
    
    def is_straight(self,path, tol=1e-3):
        # check if path is approximately straight
        x0, y0 = path[0]
        x1, y1 = path[-1]
        d_total = np.hypot(x1-x0, y1-y0)
        d_path = sum(np.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) 
                    for i in range(len(path)-1))
        return abs(d_total - d_path) < tol
    
    def parametric_path(self,path, path_type="auto", direction="ccw"):
        """
        Return parametric function x(t), y(t) for t in [0,1]
        """
        if path_type == "straight" or (path_type=="auto" and self.is_straight(path)):
            x0, y0 = path[0]
            x1, y1 = path[-1]
            return lambda t: (x0 + t*(x1-x0), y0 + t*(y1-y0))
        
        else:  # arc
            x0, y0 = path[0]
            x1, y1 = path[-1]
            # fit circle using 3 points (first, middle, last)
            xm, ym = path[len(path)//2]
            # Solve for circle center
            A = np.array([[x1-x0, y1-y0],
                        [xm-x1, ym-y1]])
            B = np.array([[(x1**2 - x0**2 + y1**2 - y0**2)/2],
                        [(xm**2 - x1**2 + ym**2 - y1**2)/2]])
            try:
                cx, cy = np.linalg.lstsq(A, B, rcond=None)[0].flatten()
            except:
                cx, cy = 0,0
            R = np.sqrt((x0-cx)**2 + (y0-cy)**2)
            # angles
            theta0 = np.arctan2(y0-cy, x0-cx)
            theta1 = np.arctan2(y1-cy, x1-cx)
            # adjust angle difference according to direction
            dtheta = theta1 - theta0
            if direction=="ccw" and dtheta < 0: dtheta += 2*np.pi
            if direction=="cw" and dtheta > 0: dtheta -= 2*np.pi
            return lambda t: (cx + R*np.cos(theta0 + t*dtheta), cy + R*np.sin(theta0 + t*dtheta))



class PathFinder():
    def __init__(self, obstacle: Obstacle, robot: Robot, tag_map: TagMap):
        self.obstacles = obstacle
        self.robot = robot
        self.tag_map = tag_map
        self.step = 0.1
        self.R = self.robot.max_steer

    def direct_planner(self, start, goal , step: float = None):
        if step is None:
            step = self.step
        n = int(np.hypot(goal[0]-start[0], goal[1]-start[1]) / step) + 2
        return [(np.linspace(start[0], goal[0], n)[i],
                 np.linspace(start[1], goal[1], n)[i]) for i in range(n)]

    def arc_planner(self,start, goal, step: float = None ,dir: str="ccw"or"cw",R: float = None):
        x1, y1, _ = start
        x2, y2, _ = goal
        dx, dy = x2-x1, y2-y1
        d = np.hypot(dx, dy)

        if R < d/2:
            R = d/2 + 1e-3

        mx, my = (x1+x2)/2, (y1+y2)/2
        h = np.sqrt(R**2 - (d/2)**2)
        nx, ny = -dy/d, dx/d
        if dir == "ccw":
            cx, cy = mx + h*nx, my + h*ny
        else:
            cx, cy = mx - h*nx, my - h*ny

        theta1 = np.arctan2(y1-cy, x1-cx)
        theta2 = np.arctan2(y2-cy, x2-cx)
        dtheta = np.arctan2(np.sin(theta2-theta1), np.cos(theta2-theta1))
        if dir == "ccw" and dtheta < 0:
            dtheta += 2*np.pi
        if dir == "cw" and dtheta > 0:
            dtheta -= 2*np.pi

        arc_len = abs(R*dtheta)
        n_pts = int(np.ceil(arc_len/step)) + 1
        path = []
        for i in range(n_pts):
            t = i/(n_pts-1)
            theta = theta1 + t*dtheta
            x = cx + R*np.cos(theta)
            y = cy + R*np.sin(theta)
            path.append((x, y))
        return path
        
    def easy_dubins_planner(self, start, goal, step: float = None, R: float = None,  dublin_types: list = None):
        if step is None:
            step = self.step
        if R is None:
            R = self.R
        sx,sy,syaw = start
        gx,gy,gyaw = goal
        try:
            path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(sx,sy,syaw,gx,gy,gyaw,1/R,selected_types=dublin_types)
            path = list(zip(path_x, path_y))
            return path
        except:
            print("Easy Dubins : No path found!")
            return None
    
    def dubins_planner(self, start, goal ,obstacle, robot_length, max_steering_angle):
        path = dubins_path.get_path(obstacle, start, goal , length=robot_length , max_steering_angle = max_steering_angle)
        if path is None:
            print("Dubins : No path found!")
            return
        else:
            return path

    def hybrid_astar_planner(self, start, goal, obstacle, heuristic: bool = True, extra_steering: bool = False):
        print('Searching ... (Hybrid Astar)')
        path = hybrid_astar.get_path([],start,goal,heuristic=True,extra_steering=True)
        if path is None:
            print("Hybrid Astar : No path found!")
            return
        else:
            return path

    def rrt_planner(self, start, goal, obstacle):
        print('Searching ... (RRT)')
        path = rrt.get_path(obs=obstacle, start_pos=start, end_pos=goal)
        if path is None:
            print("RRT : No path found!")
            return
        else:
            return path
    def mod2pi(self,theta):
        return theta - 2*math.pi*math.floor(theta/(2*math.pi))

    def dubins_path_planning(self,start, goal, turning_radius, step_size=0.1):
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        dx = gx - sx
        dy = gy - sy
        D = math.hypot(dx, dy)
        d = D / turning_radius

        theta = math.atan2(dy, dx)
        alpha = self.mod2pi(syaw - theta)
        beta = self.mod2pi(gyaw - theta)

        best_path = None
        best_length = float("inf")

        def LSL(alpha, beta, d):
            tmp = d + math.sin(alpha) - math.sin(beta)
            p2 = 2 + d**2 - 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)-math.sin(beta))
            if p2 < 0: return None
            p = math.sqrt(p2)
            t = self.mod2pi(-alpha + math.atan2((math.cos(beta)-math.cos(alpha)), tmp))
            q = self.mod2pi(beta - math.atan2((math.cos(beta)-math.cos(alpha)), tmp))
            return (t, p, q)

        def RSR(alpha, beta, d):
            tmp = d - math.sin(alpha) + math.sin(beta)
            p2 = 2 + d**2 - 2*math.cos(alpha-beta) + 2*d*(-math.sin(alpha)+math.sin(beta))
            if p2 < 0: return None
            p = math.sqrt(p2)
            t = self.mod2pi(alpha - math.atan2((math.cos(alpha)-math.cos(beta)), tmp))
            q = self.mod2pi(-beta + math.atan2((math.cos(alpha)-math.cos(beta)), tmp))
            return (t, p, q)

        def LSR(alpha, beta, d):
            p2 = -2 + d**2 + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)+math.sin(beta))
            if p2 < 0: return None
            p = math.sqrt(p2)
            tmp = math.atan2((-math.cos(alpha)-math.cos(beta)), (d+math.sin(alpha)+math.sin(beta))) - math.atan2(-2.0, p)
            t = self.mod2pi(-alpha + tmp)
            q = self.mod2pi(-beta + tmp)
            return (t, p, q)

        def RSL(alpha, beta, d):
            p2 = -2 + d**2 + 2*math.cos(alpha-beta) - 2*d*(math.sin(alpha)+math.sin(beta))
            if p2 < 0: return None
            p = math.sqrt(p2)
            tmp = math.atan2((math.cos(alpha)+math.cos(beta)), (d-math.sin(alpha)-math.sin(beta))) - math.atan2(2.0, p)
            t = self.mod2pi(alpha - tmp)
            q = self.mod2pi(beta - tmp)
            return (t, p, q)

        def RLR(alpha, beta, d):
            tmp = (6 - d**2 + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)-math.sin(beta))) / 8
            if abs(tmp) > 1: return None
            p = self.mod2pi(2*math.pi - math.acos(tmp))
            t = self.mod2pi(alpha - math.atan2((math.cos(alpha)-math.cos(beta)), d-math.sin(alpha)+math.sin(beta)) + p/2)
            q = self.mod2pi(alpha - beta - t + p)
            return (t, p, q)

        def LRL(alpha, beta, d):
            tmp = (6 - d**2 + 2*math.cos(alpha-beta) + 2*d*(-math.sin(alpha)+math.sin(beta))) / 8
            if abs(tmp) > 1: return None
            p = self.mod2pi(2*math.pi - math.acos(tmp))
            t = self.mod2pi(-alpha + math.atan2((math.cos(alpha)-math.cos(beta)), d+math.sin(alpha)-math.sin(beta)) + p/2)
            q = self.mod2pi(beta - alpha - t + p)
            return (t, p, q)

        path_types = {
            "LSL": LSL(alpha, beta, d),
            "RSR": RSR(alpha, beta, d),
            "LSR": LSR(alpha, beta, d),
            "RSL": RSL(alpha, beta, d),
            "RLR": RLR(alpha, beta, d),
            "LRL": LRL(alpha, beta, d),
        }

        for k, v in path_types.items():
            if v is None: continue
            t, p, q = v
            length = (t + p + q) * turning_radius
            if length < best_length:
                best_length = length
                best_path = (k, (t, p, q))

        if best_path is None:
            raise ValueError("No valid Dubins path found")

        path_type, (t, p, q) = best_path
        def generate_segment(x, y, yaw, seg_type, seg_length):
            points = []
            if seg_type == "S":
                n = max(1, int(seg_length*turning_radius/step_size))
                for i in range(n):
                    x += step_size*math.cos(yaw)
                    y += step_size*math.sin(yaw)
                    points.append((x, y, yaw))
                # snap final
                x += (seg_length*turning_radius - n*step_size)*math.cos(yaw)
                y += (seg_length*turning_radius - n*step_size)*math.sin(yaw)
                points.append((x, y, yaw))
            else:
                is_left = (seg_type == "L")
                n = max(1, int(seg_length*turning_radius/step_size))
                dtheta = step_size/turning_radius * (1 if is_left else -1)
                for i in range(n):
                    yaw += dtheta
                    x += step_size*math.cos(yaw)
                    y += step_size*math.sin(yaw)
                    points.append((x, y, yaw))
                # snap final
                yaw += (seg_length - n*step_size/turning_radius) * (1 if is_left else -1)
                x += (seg_length*turning_radius - n*step_size)*math.cos(yaw)
                y += (seg_length*turning_radius - n*step_size)*math.sin(yaw)
                points.append((x, y, yaw))
            return points, x, y, yaw
        x, y, yaw = 0.0, 0.0, alpha
        points = []
        for m, seg in zip(path_type, [t, p, q]):
            seg_points, x, y, yaw = generate_segment(x, y, yaw, m, seg)
            points.extend(seg_points)

        final_points = []
        for px, py, pyaw in points:
            gx = math.cos(theta)*px - math.sin(theta)*py + sx
            gy = math.sin(theta)*px + math.cos(theta)*py + sy
            final_points.append((gx, gy))
        
        return final_points, path_type, best_length
    
class Graph():
    def __init__(self,type:str,data=None):
        assert type in ["direct","bi-direct"] , "Invalid Graph type"
        self.data = data
        self.nodes = None
        self.edges = None
        if type == "direct":
            self.Graph = nx.DiGraph()
        elif type == "bi-direct":
            self.Graph = nx.Graph()
    
    def euclidean(self,p1, p2):
        return math.dist(p1, p2)
    
    def heuristic(self,u, v):
            pos_u = self.Graph.nodes[u]["pos"]
            pos_v = self.Graph.nodes[v]["pos"]
            return self.euclidean(pos_u, pos_v)

    def astar_path(self, start_id, goal_id):
        try:
            path_nodes = nx.astar_path(self.Graph, start_id, goal_id, heuristic=self.heuristic, weight="weight")
            cost = nx.path_weight(self.Graph, path_nodes, weight="weight")
            path_points = []
            for i in range(len(path_nodes) - 1):
                u, v = path_nodes[i], path_nodes[i+1]
                edge_data = self.Graph[u][v]
                if i == 0:
                    path_points.extend(edge_data["path_pts"])
                else:
                    path_points.extend(edge_data["path_pts"][1:])

            return path_nodes, cost, path_points
        
        except Exception as e:
            print(e)
            return None,e,None

   
    def initial_graph(self,graph_data: dict):
        for node in graph_data['nodes']:
            x,y,_ = node['pose']
            self.Graph.add_node(node["id"], **node, pos=(x, y))
        for edge in graph_data['edges']:
            n1, n2 = edge["from"], edge["to"]
            p1 = self.Graph.nodes[n1]["pos"]
            p2 = self.Graph.nodes[n2]["pos"]
            weight = self.euclidean(p1, p2)
            self.Graph.add_edge(n1, n2, **edge, weight=weight)




        



if __name__ == "__main__":
    node1 = TagNode("A", (2, 1, 0), 0)             # depot
    node2 = TagNode("B", (1, 3, np.deg2rad(90)), 1)     # station
    node3 = TagNode("C", (4, 2, np.deg2rad(0)), 2)     # viapoint
    nodes = [node1, node2, node3]
    tagmap = TagMap(nodes)

    obstacle = Obstacle()
    robot = Robot(length=2.0, max_speed=1.0, max_steer=41.0, current_pose=(0,0,0), turning_radius=1.5)
    path_finder = PathFinder(obstacle, robot, tagmap)

    path_dubin_AB = path_finder.hybrid_astar_planner(start=tagmap.get_tag("A").pose, goal=tagmap.get_tag("B").pose, obstacle=obstacle,heuristic=True, extra_steering=True)
    path_dubin_BC = path_finder.hybrid_astar_planner(start=tagmap.get_tag("B").pose, goal=tagmap.get_tag("C").pose, obstacle=obstacle,heuristic=True, extra_steering=True)
    path_dubin_AC = path_finder.hybrid_astar_planner(start=tagmap.get_tag("A").pose, goal=tagmap.get_tag("C").pose, obstacle=obstacle,heuristic=True, extra_steering=True)
    path_dubin_BA = path_finder.hybrid_astar_planner(start=tagmap.get_tag("B").pose, goal=tagmap.get_tag("A").pose, obstacle=obstacle,heuristic=True, extra_steering=True)

    Utils().plot_tag(tagmap)
    Utils().plot_path_with_arrows(path_dubin_AB, color='r', arrow_step=10,label='Hybrid Astar Path AB')
    Utils().plot_path_with_arrows(path_dubin_BC, color='g', arrow_step=10,label='Hybrid Astar Path BC')
    Utils().plot_path_with_arrows(path_dubin_AC, color='b', arrow_step=10,label='Hybrid Astar Path AC')
    Utils().plot_path_with_arrows(path_dubin_BA, color='y', arrow_step=10,label='Hybrid Astar Path BA')
    Utils().show()
import numpy as np
import g2o
import os
import pickle

def get_data(filename):
    """
    This funciton loads data pickled by the process_data script and removes half the odometry measurements
    so that we have the same number of laser scans and odom measurements. It also removes laser scans without the full
    361 measurements.
    """
    load_file = open(filename,'rb')
    data = pickle.load(load_file)
    skipped_vertices = []
    # Pre-filtering the data for making future processing easier
    # Check the LiDAR scans for incomplete scans to filter out
    for i, scan in enumerate(data['scans']):
        if len(scan) < 361:
            skipped_vertices.append(i)
    # Sample only half of the odom data, because it collects twice as often as lidar
    new_odom_data = []
    for k, scan in enumerate(data['odom']):
        if k % 2 == 0:
            new_odom_data.append(scan)
    data['odom'] = new_odom_data
    # Remove the incomplete scans from LiDAR and odom data
    for j in reversed(skipped_vertices):
        del data['scans'][j]
        del data['odom'][j]

    print("Number of skipped vertices: ", len(skipped_vertices))
    print("lidar length: ", len(data['scans']))
    print("length of each scan: ", len(data['scans'][0]))
    print("odom length: ", len(data['odom']))

    return data

class PoseGraphOptimization(g2o.SparseOptimizer):
    """
    This is based off the g2opy sample code. It has methods to creature g2o edges and nodes
    and run graph optimization on them.
    """
    def __init__(self):
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices, measurement,
            information=np.identity(6),
            robust_kernel=None):

        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()

    def make_graph(self, data):
        """
        This is the method we added. It takes data in the dictionary format we recorded, and
        turns it into g2o edges and nodes.
        """
        lidar = data['scans'] # list of laser scan data
        odom = data['odom'] # list of odom data
        closure = data['closures'] # index of loop closure scan in laser scans
        i = -1
        # Loop through lidar data and add each measured point as a vertex
        for k,scan in enumerate(lidar): # loop through each scan
            for point in scan: # loop through each point in each scan
                i = i+1
                # t is the pose estimate of the scanned point
                t = g2o.Isometry3d(np.identity(3),[point[0], point[1], 0])
                self.add_vertex(i, t)

        vertices = super().vertices() # save all the laser scan vertices to a variable we can use later
        print(len(vertices))

        # Loop through odom points and add them as vertices. Then, add edges between the odom point and each point in the most
        # recent laser scan.
        for f, point in enumerate(odom):
            # End the loop if there are more odom points than LiDAR, since we are only keeping even values
            if f > len(lidar): # just in case, stop when you have an odom point for each laser scan. otherwise the indexes won't match
                break
            i = i+1
            # Add odom vertices
            t = g2o.Isometry3d(np.identity(3),[point[0], point[1], 0])
            self.add_vertex(i, t)

            # Add edges between current odom point and all corresponding lidar points
            start_index = int((f) * 361)
            for x in range(361):
                lidar_pt = lidar[f][x]
                diff = g2o.Isometry3d(np.identity(3), [(point[0]-lidar_pt[0]), (point[1]-lidar_pt[1]), 0])
                self.add_edge([i, x], diff)

        # Add edge from loop closures
        diff = g2o.Isometry3d(np.identity(3)*5, [0,0,0])
        self.add_edge([closure[0], closure[1]], diff)

        print('num vertices:', len(super().vertices()))
        print('num edges: ', len(super().edges()))

if __name__ == '__main__':
    data = get_data('data_4')
    opt = PoseGraphOptimization()
    opt.make_graph(data)
    opt.save('og1.g2o') #save a g2o file of the data before optimization
    opt.optimize()
    opt.save('yay1.g2o') #save a g2o file of the data after optimization
68230

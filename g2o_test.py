import numpy as np
import g2o
import os
import pickle

def get_data(filename):
    load_file = open(filename,'rb')
    data = pickle.load(load_file)
    #print(data)
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
    for j in reverse(skipped_vertices):
        del data['scans'][j]
        del data['odom'][j]

    print("lidar length: ", len(data['scans']))
    print("length of each scan: ", len(data['scans'][0]))
    print("odom length: ", len(data['odom']))

    return data

class PoseGraphOptimization(g2o.SparseOptimizer):
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

    def make_vertices(self, data):
        lidar = data['scans']
        odom = data['odom']
        i = -1
        skipped_scans = []
        # Loop through lidar
        for k,scan in enumerate(lidar):
            # Skip the scans that are the wrong length
            if len(scan) < 361:
                skipped_scans.append(k)
                continue
            for point in scan:
                i = i+1
                # q = g2o.Quaternion(0,0,0,0)
                t = g2o.Isometry3d(np.identity(3),[point[0], point[1], 0])
                self.add_vertex(i, t)

        vertices = super().vertices()
        print(len(vertices))
        print(skipped_scans)
        # Loop through odom points
        for f, point in enumerate(odom):
            # End the loop if there are more odom points than LiDAR, since we are only keeping even values
            if f/2 > len(lidar):
                break
            # Skip the scans that were skipped for LiDAR
            if f in skipped_scans:
                continue
            if f % 2 == 0:
                i = i+1
                # q = g2o.Quaternion(0,0,0,0)
                # Add odom vertices
                t = g2o.Isometry3d(np.identity(3),[point[0], point[1], 0])
                self.add_vertex(i, t)

                # Add edges between current odom point and all corresponding lidar points
                start_index = int((f/2) * 361)
                for x in range(361):
                    # print(len(lidar[int(f/2)]))
                    # print(x)
                    lidar_pt = lidar[int(f/2)][x]
                    diff = g2o.Isometry3d(np.identity(3), [(point[0]-lidar_pt[0]), (point[1]-lidar_pt[1]), 0])
                    self.add_edge([i, x], diff)

        # Add edge from loop closures
        closure = data['closures']
        diff = g2o.Isometry3d(np.identity(3)*5, [0,0,0])
        self.add_edge([closure[0], closure[1]], diff)

        print('num vertices:', len(super().vertices()))
        print('num edges: ', len(super().edges()))

if __name__ == '__main__':
    data = get_data('data_4')
    opt = PoseGraphOptimization()
    opt.make_vertices(data)
    opt.save('og1.g2o')
    opt.optimize()
    opt.save('yay1.g2o')

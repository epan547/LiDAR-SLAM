from graphslam.load import load_g2o_se2


g = load_g2o_se2("yay.g2o")  # https://lucacarlone.mit.edu/datasets/

g.plot(vertex_markersize=1)
# g.calc_chi2()
# g.optimize()
# g.plot(vertex_markersize=1)

from graphslam.load import load_g2o_se3


g = load_g2o_se3("yay1.g2o")
g2 = load_g2o_se3("og1.g2o")

g.plot(vertex_markersize=1)
g2.plot(vertex_markersize=1)
# g.calc_chi2()
# g.optimize()
# g.plot(vertex_markersize=1)

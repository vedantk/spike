
{{{id=0|
# centroid variables
ox, oy, oz, orad = var('ox, oy, oz, orad')
///
}}}

{{{id=1|
# various arm variables
l_pin = var('l_pin')
l_pin = 5
arm1l_1, arm1l_2, arm1phi_1, arm1phi_2, arm1phi_3, arm1theta_1, arm1theta_2, arm1theta_3, arm1theta_s, arm1phi_s = var('arm1l_1, arm1l_2, arm1phi_1, arm1phi_2, arm1phi_3, arm1theta_1, arm1theta_2, arm1theta_3, arm1theta_s, arm1phi_s')
arm1theta_s = 20
arm1phi_s = 20
arm2l_1, arm2l_2, arm2phi_1, arm2phi_2, arm2phi_3, arm2theta_1, arm2theta_2, arm2theta_3, arm2theta_s, arm2phi_s = var('arm2l_1, arm2l_2, arm2phi_1, arm2phi_2, arm2phi_3, arm2theta_1, arm2theta_2, arm2theta_3, arm2theta_s, arm2phi_s')
arm2theta_s = 20
arm2phi_s = 20
///
}}}

{{{id=2|
### Goals and constraints for arm 1
arm1gx, arm1gy, arm1gz = var('arm1gx, arm1gy, arm1gz')
c1l_1min(l_1) = - l_1 - 0
c1l_1max(l_1) = l_1 - 5000000
c1l_2min(l_2) = - l_2 - 0
c1l_2max(l_2) = l_2 - 5000000
c1phi_1min(phi_1) = - phi_1 - -360
c1phi_1max(phi_1) = phi_1 - 360
c1phi_2min(phi_2) = - phi_2 - -360
c1phi_2max(phi_2) = phi_2 - 360
c1phi_3min(phi_3) = - phi_3 - -360
c1phi_3max(phi_3) = phi_3 - 360
c1theta_1min(theta_1) = - theta_1 - -360
c1theta_1max(theta_1) = theta_1 - 360
c1theta_2min(theta_2) = - theta_2 - -360
c1theta_2max(theta_2) = theta_2 - 360
c1theta_3min(theta_3) = - theta_3 - -360
c1theta_3max(theta_3) = theta_3 - 360
///
}}}

{{{id=3|
### Goals and constraints for arm 2
arm2gx, arm2gy, arm2gz = var('arm2gx, arm2gy, arm2gz')
c2l_1min(l_1) = - l_1 - 0
c2l_1max(l_1) = l_1 - 5000000
c2l_2min(l_2) = - l_2 - 0
c2l_2max(l_2) = l_2 - 5000000
c2phi_1min(phi_1) = - phi_1 - -360
c2phi_1max(phi_1) = phi_1 - 360
c2phi_2min(phi_2) = - phi_2 - -360
c2phi_2max(phi_2) = phi_2 - 360
c2phi_3min(phi_3) = - phi_3 - -360
c2phi_3max(phi_3) = phi_3 - 360
c2theta_1min(theta_1) = - theta_1 - -360
c2theta_1max(theta_1) = theta_1 - 360
c2theta_2min(theta_2) = - theta_2 - -360
c2theta_2max(theta_2) = theta_2 - 360
c2theta_3min(theta_3) = - theta_3 - -360
c2theta_3max(theta_3) = theta_3 - 360
///
}}}

{{{id=4|
# Forward kinematics: x position.
px(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3) = (orad * sin(phi_s) * cos(theta_s))+ (l_1 * sin(phi_s+phi_1) * cos(theta_s+theta_1))+ (l_2 * sin(phi_s+phi_1+phi_2) * cos(theta_s+theta_1+theta_2))+ (l_pin * sin(phi_s+phi_1+phi_2+phi_3) * cos(theta_s+theta_1+theta_2+theta_3))
///
}}}

{{{id=5|
# Forward kinematics: y position.
py(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3) = (orad * cos(phi_s))+ (l_1 * cos(phi_s+phi_1))+ (l_2 * cos(phi_s+phi_1+phi_2))+ (l_pin * cos(phi_s+phi_1+phi_2+phi_3))
///
}}}

{{{id=6|
# Forward kinematics: x position.
pz(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3) = (orad * sin(phi_s) * sin(theta_s))+ (l_1 * sin(phi_s+phi_1) * sin(theta_s+theta_1))+ (l_2 * sin(phi_s+phi_1+phi_2) * sin(theta_s+theta_1+theta_2))+ (l_pin * sin(phi_s+phi_1+phi_2+phi_3) * sin(theta_s+theta_1+theta_2+theta_3))
///
}}}

{{{id=7|
# Define objective function for single arm
f(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3, gx, gy, gz) = (vector([px(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3),py(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3),pz(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3)])- vector([gx, gy, gz])).norm()
///
}}}

{{{id=8|
# Define total objective function
F = f(arm1theta_s, arm1phi_s, arm1l_1,arm1l_2,arm1phi_1,arm1phi_2,arm1phi_3,arm1theta_1,arm1theta_2,arm1theta_3,arm1gx,arm1gy,arm1gz) + f(arm2theta_s, arm2phi_s, arm2l_1,arm2l_2,arm2phi_1,arm2phi_2,arm2phi_3,arm2theta_1,arm2theta_2,arm2theta_3,arm2gx,arm2gy,arm2gz)
///
}}}

{{{id=9|
# lagrangian variables
lambda11, lambda12, lambda13, lambda14, lambda15, lambda16, lambda17, lambda18, lambda19, lambda110, lambda111, lambda112, lambda113, lambda114, lambda115, lambda116 = var('lambda11, lambda12, lambda13, lambda14, lambda15, lambda16, lambda17, lambda18, lambda19, lambda110, lambda111, lambda112, lambda113, lambda114, lambda115, lambda116')
slack11, slack12, slack13, slack14, slack15, slack16, slack17, slack18, slack19, slack110, slack111, slack112, slack113, slack114, slack115, slack116 = var('slack11, slack12, slack13, slack14, slack15, slack16, slack17, slack18, slack19, slack110, slack111, slack112, slack113, slack114, slack115, slack116')
lambda21, lambda22, lambda23, lambda24, lambda25, lambda26, lambda27, lambda28, lambda29, lambda210, lambda211, lambda212, lambda213, lambda214, lambda215, lambda216 = var('lambda21, lambda22, lambda23, lambda24, lambda25, lambda26, lambda27, lambda28, lambda29, lambda210, lambda211, lambda212, lambda213, lambda214, lambda215, lambda216')
slack21, slack22, slack23, slack24, slack25, slack26, slack27, slack28, slack29, slack210, slack211, slack212, slack213, slack214, slack215, slack216 = var('slack21, slack22, slack23, slack24, slack25, slack26, slack27, slack28, slack29, slack210, slack211, slack212, slack213, slack214, slack215, slack216')
///
}}}

{{{id=10|
L(arm1l_1, arm1l_2, arm1phi_1, arm1phi_2, arm1phi_3, arm1theta_1, arm1theta_2, arm1theta_3, arm2l_1, arm2l_2, arm2phi_1, arm2phi_2, arm2phi_3, arm2theta_1, arm2theta_2, arm2theta_3) = F - vector([lambda11, lambda12, lambda13, lambda14, lambda15, lambda16, lambda17, lambda18, lambda19, lambda110, lambda111, lambda112, lambda113, lambda114, lambda115, lambda116, lambda21, lambda22, lambda23, lambda24, lambda25, lambda26, lambda27, lambda28, lambda29, lambda210, lambda211, lambda212, lambda213, lambda214, lambda215, lambda216]).dot_product(vector([c1l_1min(l_1), c1l_1max(l_1), c1l_2min(l_2), c1l_2max(l_2), c1phi_1min(phi_1), c1phi_1max(phi_1), c1phi_2min(phi_2), c1phi_2max(phi_2), c1phi_3min(phi_3), c1phi_3max(phi_3), c1theta_1min(theta_1), c1theta_1max(theta_1), c1theta_2min(theta_2), c1theta_2max(theta_2), c1theta_3min(theta_3), c1theta_3max(theta_3), c2l_1min(l_1), c2l_1max(l_1), c2l_2min(l_2), c2l_2max(l_2), c2phi_1min(phi_1), c2phi_1max(phi_1), c2phi_2min(phi_2), c2phi_2max(phi_2), c2phi_3min(phi_3), c2phi_3max(phi_3), c2theta_1min(theta_1), c2theta_1max(theta_1), c2theta_2min(theta_2), c2theta_2max(theta_2), c2theta_3min(theta_3), c2theta_3max(theta_3)]) + vector([slack11, slack12, slack13, slack14, slack15, slack16, slack17, slack18, slack19, slack110, slack111, slack112, slack113, slack114, slack115, slack116, slack21, slack22, slack23, slack24, slack25, slack26, slack27, slack28, slack29, slack210, slack211, slack212, slack213, slack214, slack215, slack216]))

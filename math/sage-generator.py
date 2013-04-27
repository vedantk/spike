# get sage strings
current_section = 0
def new_section():
    global current_section
    output = ''
    if current_section > 0:
        output += '///\n'
        output += '}}}\n'
    output += "\n"
    output += '{{{{{{id={0}|'.format(current_section)
    current_section += 1
    return output

def variables(str_variables):
    return str_variables + " = var('"+str_variables+"')"

def max_constraint(function_name, input_name, max_value):
    return "{0}({1}) = {1} - {2}".format(function_name, input_name, max_value)

def min_constraint(function_name, input_name, max_value):
    return "{0}({1}) = - {1} - {2}".format(function_name, input_name, max_value)

# define the person
num_arms = 2

class Arm:
    max_segment_1_length = 5000000
    max_segment_2_length = 5000000
    pin_length = 5
    def __init__(self, shoulder_phi, shoulder_theta):
        self.shoulder_phi = shoulder_phi
        self.shoulder_theta = shoulder_theta
        self.constraints = dict({
            'theta_1' : [-360, 360],
            'phi_1'   : [-360, 360],
            'l_1'     : [0, Arm.max_segment_1_length],
            'theta_2' : [-360, 360],
            'phi_2'   : [-360, 360],
            'l_2'     : [0, Arm.max_segment_2_length],
            'theta_3' : [-360, 360],
            'phi_3'   : [-360, 360]
        })

arms = []
for i in range(num_arms):
    arms.append(Arm(20, 20))
sorted_constraints = sorted(arms[0].constraints.keys())

print(new_section())
print('# centroid variables')
print(variables("ox, oy, oz, orad"))

print(new_section())
print('# various arm variables')
print(variables('l_pin'))
print('l_pin = ' + str(arms[0].pin_length))
for a, arm in enumerate(arms):
    arm_vars = ''
    for c, constraint in enumerate(sorted_constraints):
        if c > 0: arm_vars += ", "
        arm_vars += 'arm{0}'.format(a+1) + constraint
    arm_vars += ', arm{0}theta_s'.format(a+1)
    arm_vars += ', arm{0}phi_s'.format(a+1)
    print(variables(arm_vars))
    print('arm{0}theta_s = '.format(a+1) + str(arm.shoulder_theta))
    print('arm{0}phi_s = '.format(a+1) + str(arm.shoulder_phi))

for a, arm in enumerate(arms):
    print(new_section())
    print("### Goals and constraints for arm " + str(a+1))
    # goal points for each arm
    print(variables("arm{0}gx, arm{0}gy, arm{0}gz".format(a+1)))

    # constraint functions for each arm

    for constraint in sorted_constraints:
        print(min_constraint("c{0}{1}min".format(a+1, constraint), constraint, arm.constraints[constraint][0]))
        print(max_constraint("c{0}{1}max".format(a+1, constraint), constraint, arm.constraints[constraint][1]))

###
### FORWARD KINEMATICS
###

### X POSITION
print(new_section())
print('# Forward kinematics: x position.')
print('px(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3) = ', end='')

# get to shoulder
print('(orad * sin(phi_s) * cos(theta_s))', end='')

# to segment 1
print('+ (l_1 * sin(phi_s+phi_1) * cos(theta_s+theta_1))', end='')

# to segment 2
print('+ (l_2 * sin(phi_s+phi_1+phi_2) * cos(theta_s+theta_1+theta_2))',end='')

# to end
print('+ (l_pin * sin(phi_s+phi_1+phi_2+phi_3) * cos(theta_s+theta_1+theta_2+theta_3))')

### Y POSITION
print(new_section())
print('# Forward kinematics: y position.')
print('py(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3) = ', end='')

# get to shoulder
print('(orad * cos(phi_s))', end='')

# to segment 1
print('+ (l_1 * cos(phi_s+phi_1))', end='')

# to segment 2
print('+ (l_2 * cos(phi_s+phi_1+phi_2))',end='')

# to end
print('+ (l_pin * cos(phi_s+phi_1+phi_2+phi_3))')

### Z POSITION
print(new_section())
print('# Forward kinematics: x position.')
print('pz(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3) = ', end='')

# get to shoulder
print('(orad * sin(phi_s) * sin(theta_s))', end='')

# to segment 1
print('+ (l_1 * sin(phi_s+phi_1) * sin(theta_s+theta_1))', end='')

# to segment 2
print('+ (l_2 * sin(phi_s+phi_1+phi_2) * sin(theta_s+theta_1+theta_2))',end='')

# to end
print('+ (l_pin * sin(phi_s+phi_1+phi_2+phi_3) * sin(theta_s+theta_1+theta_2+theta_3))')



###
### OBJECTIVE FUNCTION
###

print(new_section())
print('# Define objective function for single arm')
print('f(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3, gx, gy, gz) = ', end='')
print('(vector([\
px(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3),\
py(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3),\
pz(theta_s, phi_s, l_1, l_2, phi_1, phi_2, phi_3, theta_1, theta_2, theta_3)])\
- vector([gx, gy, gz])).norm()')

print(new_section())
print('# Define total objective function')
total_objective = ''
total_objective_template = 'f(arm{0}theta_s, arm{0}phi_s, arm{0}l_1,arm{0}l_2,arm{0}phi_1,arm{0}phi_2,arm{0}phi_3,arm{0}theta_1,arm{0}theta_2,arm{0}theta_3,arm{0}gx,arm{0}gy,arm{0}gz)'
for a, arm in enumerate(arms):
    if (total_objective): total_objective += " + "
    total_objective += total_objective_template.format(a+1)
print('F = ' + total_objective)

###
### LAGRANGIAN
###
print(new_section())
print('# lagrangian variables')
all_lambdas = ''
all_slacks = ''
for a, arm in enumerate(arms):
    lambdas = ''
    slacks  = ''
    for c, constraint in enumerate(sorted_constraints):
        if lambdas: lambdas += ', '
        if slacks: slacks += ", "

        # double lambdas/slacks for max and min constraints
        lambdas += 'lambda{}{}, '.format(a+1, 2*c + 1)
        lambdas += 'lambda{}{}'.format(a+1, 2*c + 2)
        slacks += 'slack{}{}, '.format(a+1, 2*c + 1)
        slacks += 'slack{}{}'.format(a+1, 2*c + 2)

    if all_lambdas: all_lambdas += ", "
    all_lambdas += lambdas

    if all_slacks: all_slacks += ", "
    all_slacks += slacks

    print(variables(lambdas))
    print(variables(slacks))

print(new_section())
lagrangian = 'L('
lagrangian_parameters = ''
constraints_vector = ''
for a, arm in enumerate(arms):
    for c, constraint in enumerate(sorted_constraints):
        if lagrangian_parameters: lagrangian_parameters += ", "
        if constraints_vector: constraints_vector += ", "

        lagrangian_parameters += 'arm{}'.format(a+1) + constraint
        constraints_vector += 'c{0}{1}min({1}), c{0}{1}max({1})'.format(a+1, constraint)
lagrangian += lagrangian_parameters + ") = "

lagrangian += 'F - vector([{lambdas}]).dot_product(vector([{constraints}]) + vector([{slacks}]))'.format(lambdas=all_lambdas, constraints=constraints_vector, slacks=all_slacks);
print(lagrangian)

# code for problem set 1 of 2D grid map
# global variables
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up
def sense(p, colors, measurement):
    s = 0.0
    q = [[0.0 for row in range(len(colors[0]))] for col in range(len(colors))]
    for i in range(len(p)):
        for j in range(len(p[i])):
            hit = (measurement == colors[i][j])
            x = p[i][j] * (hit * sensor_right + (1-hit) * sensor_wrong)
            q[i][j] = x
            s = s + q[i][j]
    for i in range(len(p)):
        for j in range(len(p[i])):
            q[i][j] = q[i][j] / s
    return q

def move(p, motion):
    n = len(p)
    m = len(p[0])
    q = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]
    for i in range(len(p)):
        for j in range(len(p[i])):
            s = p[(i - motion[0]) % n][(j - motion[1]) % m] * p_move
            s = s + p[i][j] * p_stay
            q[i][j] = s
    return q

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']'
    
#############################################################
# For the following test case, your output should be 
# [[0.01105, 0.02464, 0.06799, 0.04472, 0.02465],
#  [0.00715, 0.01017, 0.08696, 0.07988, 0.00935],
#  [0.00739, 0.00894, 0.11272, 0.35350, 0.04065],
#  [0.00910, 0.00715, 0.01434, 0.04313, 0.03642]]
# (within a tolerance of +/- 0.001 for each entry)
colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]
measurements = ['G','G','G','G','G']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]
sensor_right = 0.7
sensor_wrong = 1 - sensor_right
p_move = 0.8
p_stay = 1 - p_move
# check data
if len(measurements) != len(motions):
    raise ValueError, 'error in size of measurement/motion vector'

pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]

for k in range(len(measurements)):
    p = move(p, motions[k])
    p = sense(p, colors, measurements[k])
show(p)





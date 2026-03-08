import matplotlib.pyplot as plt
import numpy as np

def init():
    g = 9.81
    m = 0.1
    L = 1
    theta = np.deg2rad(30.0) 
                 #mcp onyl works accurately at small angle bc its linearized 
                 # can still work up to around 30 but sketchy (sintheta)=theta approx
                 # gets worse as theta increases
                 #later, try starting vertically stable, downwards
    dot_theta = 0
    #ddot_theta = ((3*g)/(2*L))*np.sin(theta)

    dt = 0.01
    horizon = 67
    total_steps = 500
    
    #np array bc a normal python array is a nested list and does not follow w typical
    #array math
    
    A = np.array([
        [0.0, 1.0],
        [(3*g)/(2*L), 0.0]
        ])
    B = np.array([
        [0.0],
        [3/(m*L*L)]
        ])
    x = np.array([
        [theta],
        [dot_theta]
        ])
    #dot_x = np.array([
    #    [dot_theta],
    #    [ddot_theta]
    #])

    #need to turn dot_x = Ax + Bu into something i can actually use to find
    #the next state

    #derived next_x = (xidentity + dt(A))x + (dt(B))u
    #                        ^Ad               ^Bd
    
    x_id = np.array([
        [1, 0],
        [0, 1]
        ])

    Ad = x_id + dt*A
    Bd = dt*B

    u = 0.0
    #dot_x = A @ x + B * u

    ##############################
    # cost matrices, constraints #
    ##############################
    #punish angle

    c_theta = 200.0
    c_dot_theta = 10.0
    c_input = 0.5
    cf_theta = 400.0
    cf_dot_theta = 20.0

    u_max = 0.25 #(N*m)
    
    Q = np.array([
        [c_theta,     0.0],
        [0.0, c_dot_theta]
        ])
    R = np.array([ #keep this as an arr bc matrix math and shape and code expect an arr
        [c_input] 
        ])
    Qf = np.array([
        [cf_theta,      0.0],
        [0.0,  cf_dot_theta]
        ])

    ##############################
    # create prediction matrices #
    ##############################
    #find M and C for structure x = Mx0 + Cu
    
    #initialize matricies with 0
    M = np.zeros((2*horizon, 2)) #ea/ state takes 2 rows to match states (theta&v_angle)
                        #and 2 columns (effect of ea/ state from both theta0 & v_angle0)
    C = np.zeros((2*horizon, horizon)) #added effect of input to each state's properties

    for j in range(horizon):
        #run once for each predicted step state
        #j=0,(0:2) filling x1, j=1,(2:4) filling x2
        x_n1 = j*2
        x_n2 = (j+1)*2
        
        M[x_n1:x_n2,:] = np.linalg.matrix_power(Ad, j+1) 
        # ^ from x_n1 up to x_n2, all columns  ^ if j=0, Ad^1 | j=1, Ad^2 | j=2, Ad^3 ...

        #m becomes...
        #[Ad],[Ad^2],[Ad^3]
        #where Ad is also a 2x2 matrix used to find the nxt state from cur state alone
 
        for k in range(j+1):
            #test0: j=0, k range(0->1), u_n1=0 u_n2=1 C[0:2,0:1] = [Bd]
            #test1: j=1, k range(0->2), k=0 | u_n1=0, u_n2=1 C[2:4,0:1] = [AdBd]
            #                         , k=1 | u_n1=1, u_n2=2 C[2:4,1:2] = [Bd]
            u_n1 = k
            u_n2 = k+1
            C[x_n1:x_n2,u_n1:u_n2] = (np.linalg.matrix_power(Ad, j-k) @ Bd)

    ########################
    # create cost matrices #
    ########################
    #find Q and R for cost function J1 = Qx1 + Ru1, add them all up while subbing states 
    #to get cost function to minimize
    Q_m = np.zeros((2*horizon, 2*horizon))
    R_m = np.zeros((horizon, horizon))
    
    #apparently i can reuse the var for for loops as long as they arent nested
    #didnt know that, was scared i was running out of 1 letter variables lol
    for i in range(horizon):
        Q_m[2*i:2*(i+1),2*i:2*(i+1)] = Q
    Q_m[2*(horizon-1):2*horizon,2*(horizon-1):2*horizon] = Qf
    
    for i in range(horizon):
        R_m[i:i+1,i:i+1] = R
    #these are both just diagonals so theyre easy to write    


    return (g,L,m,theta,dot_theta,dt,horizon,total_steps,A,B,x,x_id,u,u_max,M,C,Q_m,R_m)

def mpc_step (x,horizon,u_max,M,C,Q_m,R_m):
    #substitute x_next = Mx + CU into J X^tQmX + U^tRmU 
    #after deriving to find a quadratic equation of input U for J, find min
    #A_j(u) + B_j = partialdJ/partialdu = 0

    A_j = C.T @ Q_m @ C + R_m
    B_j = C.T @ Q_m @ M @ x

    u_j = -np.linalg.solve(A_j,B_j) 
    #solves for u_j (A_j(u_j) = -(B_j)) -> u_j = -(B_j)/(A_j)
    #solving for u_j when partialdj = 0 finds the least costly moves
    #negative torque means it acts against the error from setpoint, like springs
    #x = -(np.linalg.solve(5,10)) -> 5x = -10, x = -2
    
    u_j = np.clip(u_j, -u_max, u_max) 
    #clamp out of bounds torque to fit to bounds, controller will only notice post-step
    #suboptimal

    #inequality within function

    return (u_j[0,0])

def plant_sim (x,u,g,L,m,dt): 
    angle = x[0, 0]
    v_angle = x[1, 0] 
    #gets angle and v_angle from state vector x-[2x1]

    a_angle = ((3*g)/(2*L))*np.sin(angle) + (3/(m*L*L))*u 
    #calc a_angle based on dynamics, (state+input)

    x_next = np.array([
        [angle + dt*v_angle],
        [v_angle + dt*a_angle]
    ])

    return(x_next)

def main():
    g,L,m,theta,dot_theta,dt,horizon,total_steps,A,B,x,x_id,u,u_max,M,C,Q_m,R_m = init()  

    #later, implement while loop, make theta start veritcally stable (=np.pi) and
    #implement swing-up control until its small angle then switch to mcp, would be cool
    
    #############################################
    theta_hist = []
    u_hist = []
    print("initial x:", x)
    #############################################

    for i in range(total_steps):

        #########################################
        theta_hist.append(np.rad2deg(x[0,0]))
        #########################################

        u = mpc_step(x,horizon,u_max,M,C,Q_m,R_m)

        #########################################
        u_hist.append(u)
        #########################################
        
        x = plant_sim(x,u,g,L,m,dt)

    ############################################
    # sim, plot whatever 
    #didnt have time to do this (10:05 am lol) so i had claude do it

    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.plot(theta_hist)
    ax1.set_ylabel('angle (deg)')
    ax2.plot(u_hist)
    ax2.set_ylabel('torque (Nm)')
    plt.xlabel('timestep')
    plt.show()

    ###########################################

    return

main()

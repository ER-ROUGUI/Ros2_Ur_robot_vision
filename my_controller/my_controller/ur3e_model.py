
import numpy as np
import math
import cv2


# def tableDHM(q):
  
#           #  "- q = array([q1,q2,q3,q4,q5,q6])"
#     tableDHM_out = np.array([ 
#                             [np.pi/2,    0,        q[0] ,    0.15185],
#                             [0,         -0.24355 , q[1] ,    0],
#                             [0,         -0.2132,   q[2] ,    0],
#                             [np.pi/2,    0,        q[3] ,    0.13105],
#                             [-np.pi/2,   0,        q[4] ,    0.08535],
#                             [0,          0,        q[5] ,    0.0921], # Same DHM parameters as Q2
#                             [0,          0,         0 ,      0.1] # ADD TOOLS DHM PARAMETERS HERE
#     ])

#     return tableDHM_out
def tableDHM(q):
  
          #  "- q = array([q1,q2,q3,q4,q5,q6])"
    tableDHM_out = np.array([ 
                            [q[0]  ,  0,       0.15185 ,    np.pi/2],
                            [q[1]  , -0.24355 ,  0      ,    0],
                            [q[2]  , -0.2132,  0      ,    0],
                            [q[3]  ,  0,       0.13105 ,   np.pi/2],
                            [q[4]  ,  0 ,      0.08535 ,   -np.pi/2],
                            [q[5]  ,  0 ,      0.0921  ,   0]]) # Same DHM parameters as Q2
                           # ADD T OOLS DHM PARAMETERS HERE
    

    return tableDHM_out

# def gi_i1(alpha_i, d_i, theta_i, r_i):
    


#     gi_i1_out=np.array([[np.cos(theta_i), -np.sin(theta_i), 0 ,d_i],
#                         [np.cos(alpha_i)*np.sin(theta_i) , np.cos(alpha_i)*np.cos(theta_i) , -np.sin(alpha_i) , -r_i*np.sin(alpha_i)],
#                         [np.sin(alpha_i)*np.sin(theta_i) , np.sin(alpha_i)*np.cos(theta_i) , np.cos(alpha_i) , r_i*np.cos(alpha_i)],
#                         [0,0,0,1]])
                 

#     return gi_i1_out

def gi_i1(theta_i, d_i, r_i,alpha_i ):
    


    gi_i1_out=np.array([[np.cos(theta_i),                   -np.sin(theta_i)*np.cos(alpha_i)    , np.sin(theta_i)*np.cos(alpha_i) ,   d_i*np.cos(theta_i)],
                        [np.sin(theta_i) ,                   np.cos(alpha_i)*np.cos(theta_i) ,  -(np.cos(theta_i))*np.sin(alpha_i) ,   d_i*np.sin(theta_i)],
                        [0 ,                                  np.sin(alpha_i) ,                  np.cos(alpha_i) ,                                    r_i],
                        [0,                                   0,                                 0,                                                    1]])
                 

    return gi_i1_out



def list_gi_i1(q_cur):

    # Inputs from functions defined previously
    tableDHM_q_cur = tableDHM(q_cur)
    Nb_rows_TableDHM = tableDHM_q_cur.shape[0]
    list_g_i_1_i = []

        
    for i in range(Nb_rows_TableDHM):
        g_i_1_i = gi_i1(tableDHM_q_cur[i][0], tableDHM_q_cur[i][1], tableDHM_q_cur[i][2], tableDHM_q_cur[i][3])
        list_g_i_1_i.append(g_i_1_i)
            

    return list_g_i_1_i

def list_g_0i(q_cur):
    

    # Inputs from functions defined previously
    list_gi_i1_out = list_gi_i1(q_cur)
    
    tableDHM_q_cur = tableDHM(q_cur)
    Nb_rows_TableDHM = tableDHM_q_cur.shape[0]
    
    list_g_0i_out=[]
    list_g_0i_out.append(list_gi_i1_out[0])
    for i in range(1,Nb_rows_TableDHM):
        list_g_0i_out.append(list_g_0i_out[-1].dot(list_gi_i1_out[i]))
        
    return list_g_0i_out

qd =np.array([0, -1.74532925 , 0 ,  -1.74532925 , 0  ,   0])
# print(list_g_0i(qd))
g0E = list_g_0i(qd)
#print(g0E[5])

def DescribeToolFrame(q):
   
    # Inputs from functions defined previously
    list_g_0i_out = list_g_0i(q)
    g_0E = list_g_0i_out[-1]

    P = g_0E[:3, 3]  
    n = g_0E[:3, 2]  
    theta = math.acos(g_0E[0, 0])
    # Computation of P,n and theta
    # To be completed... 

    return([P,n,theta])

#print(DescribeToolFrame(qi))
############################################################" kinematic model ur3e "
numberJoints = 6
jointsType = ['R','R','R','R','R','R']
def ComputeJac(q_cur):
   
  
   # Inputs
    global numberJoints, jointsType # Input variables that come from outside this function
    list_g_0i_out = list_g_0i(q_cur)
    Zi=[0,0,1]
    P0N=list_g_0i_out[-1][0:3,3]
    J=np.zeros((6,6))
    for i in range(numberJoints):
        ROi= list_g_0i_out[i][0:3,0:3]
        POi =list_g_0i_out[i][0:3,3]
        Ji= np.cross(ROi.dot(Zi),P0N-POi)
        J[:,i] = np.concatenate((Ji,ROi.dot(Zi)), axis=0)
       
        
    return J

# qi =np.array([-np.pi/2,0,-np.pi/2 ,-np.pi/2 ,-np.pi/2 ,-np.pi/2])
# #meme chose pour qf
# q_dot=np.array([0.5, 1, -0.5 ,0.5 ,1, -0.5])
# Twist1 =ComputeJac(qi).dot(q_dot)
# print(Twist1)


####################################  inverse geomitrical model


def ComputeIGM(X_d, q_0, k_max, eps_x):
             
                
        
         list_q_IGM = [q_0]  
         q = q_0
         k = 0
         while k < k_max:
               gON = list_g_0i(q)[-1] 
               # print(gON) 
               X_cur = gON[:3,3]
               error = X_d - X_cur  # error
               if np.linalg.norm(error) < eps_x:  
                  break
               J = ComputeJac(q)[:3,:]
               J_pseu_inv = np.linalg.pinv(J) 
            #    error = np.concatenate((error, [1]))
               delta_q = J_pseu_inv.dot(error) 
               q = q + delta_q  
               
               list_q_IGM.append(q)  
               
               k = k + 1

         # Results
         return list_q_IGM[-1]

####################################  inverse kinematic model
def ComputeIKM(X_d_i, X_d_f, V, Te, q_i, k_max, eps_x):    
   
    displacement = X_d_f - X_d_i
    
    distance = np.linalg.norm(displacement)
    
    total_time = distance / V
    
    num_steps = int(total_time / Te)
    
    discreteTime = np.linspace(0, total_time, num_steps)
    
    list_X_d_k = [X_d_i + (displacement * t / total_time) for t in discreteTime]
 
    list_q_dk = [q_i]

    for X_d_k in list_X_d_k:
        q_dk = ComputeIGM(X_d_k, list_q_dk[-1], k_max, eps_x)
        list_q_dk.append(q_dk)
    
    return([discreteTime, list_X_d_k, list_q_dk])

######################################### End - effector / Camera transformation 

def endeffector_camera():
    # Rotation matrix around z-axis by 45 degrees
    transformation_matrix = np.array([
                                    [np.cos(np.pi/6), -np.sin(np.pi/6), 0 , 0],
                                    [np.sin(np.pi/6), np.cos(np.pi/6), 0 , 0],
                                    [0, 0, 1 , 0.15],
                                    [0, 0, 0 , 1]
                                    ])



    return transformation_matrix

# Example usage
# result_matrix = endeffector_camera()
# print("Transformation Matrix:")
# print(result_matrix)

######################################"
def skew_matrix(vector):
    if vector.shape == (3, ) :
        return np.array([
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ])
    else:
        raise ValueError("Input vector must be a 3x1 or 1x3 numpy array.")

##########################"" Normalisation 
    
def normalized_coordinates(x_pixel, y_pixel, fx, fy ,u0 ,v0):
    x_normalized = (fx * x_pixel)  + u0
    y_normalized = (fy *y_pixel) +v0
    return x_normalized, y_normalized

##################################### interaction matrix
    
def interaction_matrix(normalised_cordo, depth):

    number_point = len(normalised_cordo)
    Ls = []  # Initialize an empty list

    for i in range(0, number_point):
        x= normalised_cordo[i][0]
        y= normalised_cordo[i][1]
        z = depth[i]

        row1 = [-1/z, 0, x/z, x*y, -(1+x**2), y]
        row2 = [0, -1/z, y/z, 1+y**2, -x*y, -x]

        # Append rows to the list
        Ls.append(row1)
        Ls.append(row2)

    return np.array(Ls)


def inv_interaction(matrix):
    try:
        inverse_matrix = np.linalg.inv(matrix)
        return inverse_matrix
    except np.linalg.LinAlgError:
        pseudo_inverse_matrix = np.linalg.pinv(matrix)
        return pseudo_inverse_matrix

s1 = [[0.02, 0.03], [0.05, 0.01], [0.04, 0.03]]
z = [1, 1, 1]

################################ comput error

def calculate_errors(circles, fixed_points):
    num_circles = len(circles)
    num_fixed_points = len(fixed_points)

    errors = np.zeros((6, 1))

    for i, circle in enumerate(circles):
        center = (circle[0], circle[1])

        if i < num_fixed_points:
            fixed_point = fixed_points[i]
            errors[i*2:i*2+2, 0] = np.array([center[0] - fixed_point[0], center[1] - fixed_point[1]])

    return errors
circles = [(100, 100, 10), (200, 200, 15), (300, 300, 20)]
fixed_points = [(90, 90), (190, 190), (290, 290)]
errors = calculate_errors(circles, fixed_points)

# Affichage des erreurs
print("Erreurs :")
print(errors)


########################" loi de commande"

# cap = cv2.VideoCapture(1)

# fixed_points = [
#     (100, 100),
#     (300, 100),
#     (100, 400),
#     (300, 400),
# ]

# # Parameters for circle detection
# param1 = 50
# param2 = 30
# min_radius = 10
# max_radius = 50
# # Initialiser la liste des cercles fixes
# fixed_circles = np.array(fixed_points)


# tolerance = 1e-6
# max_iterations = 100
# iteration = 0

    
# q_test = np.array([-np.pi/2,0,-np.pi/2 ,-np.pi/2 ,-np.pi/2 ,-np.pi/2])
# lamda = 0.2

# while iteration < max_iterations:

    

#     ret, img = cap.read()

#     if not ret:
#         print("Erreur: Impossible de capturer la vidéo.")
#         break

#     # Convertir l'image en niveaux de gris
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
#     circles = cv2.HoughCircles(
#         gray_blurred,
#         cv2.HOUGH_GRADIENT,
#         dp=1,
#         minDist=20,
#         param1=50,
#         param2=30,
#         minRadius=10,
#         maxRadius=50
#     )

#     # Si des cercles sont trouvés
#     if circles is not None:
#         circles = np.uint16(np.around(circles))[0, :]
#         circles = sorted(circles, key=lambda x: x[2], reverse=True)[:4]

#         # Calculer les erreurs avec les positions des cercles détectés
#         # errors = calculate_errors(circles, fixed_circles)

#         # # Calculer la Jacobienne inverse
#         # Jacob_inverse = np.linalg.inv(ComputeJac(q_test))

#         # # Votre code de calcul de l'erreur (à ajuster en fonction de votre application)
#         # current_error = np.mean(errors, axis=0)

#         # if np.linalg.norm(current_error) < tolerance:
#         #     print("Convergence atteinte!")
#         #     break


#         Jacob_inverse = np.linalg.inv(ComputeJac(q_test))
#         #print(np.shape(Jacob_inverse))
#         G0_E = list_g_0i(q_test)[5]
#         End_effector_Base_Rot =  G0_E[:3, :3]
#         zero_matrix = np.zeros((3, 3))

#         # End_effector_Base_Transformation = np.block([[End_effector_Base_Rot, np.zeros_like(End_effector_Base_Rot)], [np.zeros_like(End_effector_Base_Rot), End_effector_Base_Rot]])

#         # Camera_end_Effector = np.block([[endeffector_camera()[:3, :3], np.dot(skew_matrix(endeffector_camera()[:3, 3]),endeffector_camera()[:3, :3])], [np.zeros_like(End_effector_Base_Rot), endeffector_camera()[:3, :3]]])

#         # current_error = calculate_errors()
#         # if np.linalg.norm(current_error) < tolerance:
#         #     print("Convergence atteinte!")
#         #     break

#         # q_dot = -lamda * np.dot(np.dot(np.dot(np.dot(Jacob_inverse, End_effector_Base_Transformation), Camera_end_Effector), inv_interaction(interaction_matrix(s1, z))), current_error)

#         # Mettre à jour les coordonnées des joints
#         # q_test = q_test + q_dot

#     # Incrémenter le compteur d'itérations
#     iteration += 1

#     if circles is not None:
#         for circle in circles:
#             center = (circle[0], circle[1])
#             radius = circle[2]
#             cv2.circle(img, center, radius, (0, 255, 0), 2)

#     # Draw fixed points (circles) on the image
#     for point in fixed_points:
#         cv2.circle(img, point, 5, (0, 0, 255), -1)

    

#     cv2.imshow('Camera Stream', img)

#     # Attendre 1 milliseconde et vérifier si l'utilisateur appuie sur la touche 'q' pour quitter
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
# #print(np.shape(inv_interaction(interaction_matrix(s1,z))))
# # errour = np.array([1,2,3,4,5,6])

# # q_dot  = -lamda * np.dot(np.dot(np.dot(np.dot(Jacob_inverse,End_effector_Base_Transformation), Camera_end_Effector),inv_interaction(interaction_matrix(s1,z))),errour)

# # print(q_dot)
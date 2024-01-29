
import cv2
import requests
import numpy as np

# URL de la caméra
camera_url = "http://192.168.1.102:4242/current.jpg?annotations=on/off"

# Paramètres pour la détection de cercles
param1 = 60  # Augmenté pour une détection plus stricte
param2 = 50  # Augmenté pour une détection plus stricte
min_radius = 10
max_radius = 50

# Nombre de cercles à détecter
num_circles = 4

# Paramètres pour le flou gaussien
gaussian_blur_kernel_size = (11, 11)  # Taille du noyau du filtre gaussien


while True:
    try:
        # Récupérer l'image depuis l'URL
        response = requests.get(camera_url)
        img_array = np.array(bytearray(response.content), dtype=np.uint8)
        img = cv2.imdecode(img_array, -1)

        height, width, _ = img.shape
        # Ajouter un point fixe au milieu de l'image
        fixed_middle_point = (width // 2, height // 2)

        # Dessiner le point fixe au milieu de l'image en rouge
        cv2.circle(img, fixed_middle_point, 5, (0, 0, 255), -1)

        # Convertir l'image en niveaux de gris
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Appliquer une opération de flou gaussien pour réduire le bruit
        gray_blurred = cv2.GaussianBlur(gray, gaussian_blur_kernel_size, 0)

        # Utiliser la détection de cercles de Hough
        circles = cv2.HoughCircles(
            gray_blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=30,  # Ajusté pour éviter la détection multiple des cercles
            param1=param1,
            param2=param2,
            minRadius=min_radius,
            maxRadius=max_radius
        )

        # Si des cercles sont trouvés, les trier par rayon et prendre les 4 plus grands
        if circles is not None:
            circles = np.uint16(np.around(circles))[0, :]
            circles = sorted(circles, key=lambda x: x[2], reverse=True)[:num_circles]

            # Dessiner les cercles sur l'image
            for circle in circles:
                center = (circle[0], circle[1])
                radius = circle[2]
                # Dessiner le cercle
                cv2.circle(img, center, radius, (0, 255, 0), 2)

                # Afficher les coordonnées en pixels
                print(f"Position du cercle - x: {center[0]}, y: {center[1]}, rayon: {radius}")

        # Afficher l'image
        cv2.imshow('Camera Stream', img)

        # Attendre 1 milliseconde et vérifier si l'utilisateur appuie sur la touche 'q' pour quitter
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"Erreur: {e}")

# Fermer la fenêtre d'affichage et libérer les ressources
cv2.destroyAllWindows()

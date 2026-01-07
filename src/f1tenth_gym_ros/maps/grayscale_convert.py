import cv2

# Path setting
input_image_path = 'map_real_2025-02-11'+'.png'  # Nom de l'image en entrée (non grayscale)
output_image_path = 'map_real_2025-02-11'+'.png' # Nom de l'image en sortie, mettre le même pour écraser


def convert_to_grayscale(input_path, output_path):
    # Load image
    img = cv2.imread(input_path)

    # Convert image in grayscale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Save the new image
    cv2.imwrite(output_path, img_gray)



convert_to_grayscale(input_image_path, output_image_path)


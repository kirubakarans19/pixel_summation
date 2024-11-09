# import cv2

# cap = cv2.VideoCapture(0)

# def getImg(display=False,size=[480,240]):
#     _,img = cap.read()
#     img=cv2.resize(img,size[0],size[1])
#     if display:
#         cv2.imshow('IMG',img)
#     return img
# if __name__ =='__main__':
#     while True:
#         img = getImg(True)
import cv2
import subprocess

def getImg(display=False, size=[480, 240]):
    # Use fswebcam to capture an image and save it as 'img.jpg'
    subprocess.run(["fswebcam", "--no-banner", "-r", f"{size[0]}x{size[1]}", "img.jpg"])
    
    # Read the image using OpenCV
    img = cv2.imread("img.jpg")
    
    if img is None:
        print("Error: Failed to capture image.")
        return None
    
    if display:
        cv2.imshow('IMG', img)
        cv2.waitKey(1)
        
    return img

if __name__ == '__main__':
    while True:
        img = getImg(True)

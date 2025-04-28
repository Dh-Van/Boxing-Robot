import cv2
import time
cap = cv2.VideoCapture(0)  # Try different indices

if not cap.isOpened():
    print("Cannot open camera")
    exit()

ret, frame = cap.read()
# frame = cv2.cvtColor(frame, cv2.COLOR_YUV2RGB)

if ret:
    cv2.imwrite("test_image.png", frame)
    print("Image saved")
else:
    print("Failed to capture image")

cap.release()
# num = 1
# cap = cv2.VideoCapture(0)
# while True:
#     ret, img = cap.read()
#     # cv2.imshow('Frame', img)
#     if cv2.waitKey(1) & 0xFF == ord('c'):
#         cv2.imwrite('../media/images/'+str(num)+'jpg', img)
#         print('Capture '+str(num)+' Successful!')
#         num = num + 1
#     if num == 4:
#         break
# cap.release()
cv2.destroyAllWindows()
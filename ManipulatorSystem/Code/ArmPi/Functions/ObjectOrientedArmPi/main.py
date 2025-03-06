from Perception import Perception
from Motion import Motion

def main():
    perception = Perception()
    motion = Motion()

    __target_color = ('red', 'green', 'blue')
    while True:
        img = perception._get_frame()
        if img is not None:
            Frame = perception.run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    cv2.destroyAllWindows()

if(__name__ == "__main__"):
    main()
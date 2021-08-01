import numpy as np
import cv2
import mediapipe as mp
from threading import Thread
import time
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

class OMC():
    def __init__(self, videoPath, videoSize = None, skip:int = 0, QSize:int = 64) -> None:
        self.hands = mp_hands.Hands(max_num_hands=1)
        self.stream = cv2.VideoCapture(videoPath)
        self.size = videoSize
        self.buffer = []
        self.bufferSize = QSize
        self.stop = False
        self.frameCount = 0
        self.latestFrame = None
        self.last = False

        if skip != 0:
            self.__skipFrame(skip)

        self.t = Thread(target=self.__update)
        self.t.start()
    
    def __skipFrame(self, skip):
        i = 0
        while i < skip:
            ret, frame = self.stream.read()
            if ret:
                i += 1

    def __update(self):
        while not self.stop:
            if len(self.buffer) >= self.bufferSize:
                continue

            success, self.latestFrame = self.stream.read()
            
            if success:
                self.frameCount += 1
                if self.size is not None:
                    self.buffer.append(cv2.resize(self.latestFrame, self.size))
                    continue
                self.buffer.append(self.latestFrame)   
        self.last = True
        self.stream.release()

    def getFrame(self, guarantee=True):
        if guarantee:
            while len(self.buffer) == 0 and not self.last:
                pass
        if len(self.buffer) == 0 or self.last:
            return False, None
        return True, self.buffer.pop(0)

    def close(self):
        self.stop = True 

    def handDetect(self):
        '''
        this method find hands in a frame
        '''
        
        status, image = self.getFrame()
        if not status:
            return False, None
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        imageRGB.flags.writeable = False
        results = self.hands.process(imageRGB)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            status = True
        return status, image

if __name__ == "__main__":
    paths = [
                # r"resource\IMG_3735.MOV",
                # r"resource\MVI_2970.MP4"

                # r"resource\IMG_3736.MOV",
                # r"resource\MVI_2971.MP4" 
                
                # r"resource\IMG_3881.MOV",
                # r"resource\MVI_2973.MP4",
                
                r"resource\can.mp4",
                r"resource\sam.mp4"
            ]

    skips = [
                # 0,
                # 43

                # 57,
                # 0
                
                # 0,
                # 100
                
                412,
                385
            ]

    # vsize = (1280, 720)
    # vsize = (640, 360)
    vsize = None

    cap_list = [OMC(path, vsize, skips[idx]) for idx, path in enumerate(paths)]

    if vsize is not None:
        v_out_dimensions = (vsize(0)*2, vsize(1))
    out_vid = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc('M','P','4','2'), 59.94, (2560, 720))

    time.sleep(5)

    idx = 0
    # total is the number of frame we want to perform mo-cap
    total = 145
    tStart = time.time()
    while True:
        # you cannot use these two line below at the same time
        [rets, frames] = np.array([cap.handDetect() for cap in cap_list]).T       # frame with hand landmarks
        # [rets, frames] = np.array([cap.getFrame() for cap in cap_list]).T         # frame without hand landmarks
        
        # check if we have 2 frame from our capture
        if np.all(rets):
            # combine frames with hstack function
            hstack_frames = np.hstack(frames)
            
            # show hstacked frame
            cv2.imshow("preview", cv2.resize(hstack_frames, (1280, 360)))
            
            # write hstacked frame to video we created earlier
            out_vid.write(np.hstack(frames))
            
            # show verbose
            idx += 1
            print("{0:.2%}".format(idx/total), end = "\r")

        if cv2.waitKey(1) & 0xFF == ord('q') or idx >= total:
            cv2.destroyAllWindows()
            break

    # show time used
    tEnd = time.time()-tStart
    print(f"{tEnd//60}m {tEnd%60}s")

    #properly close all of the video objects and threads
    for cap in cap_list:
        cap.close()

    # finalize the video file
    out_vid.release()
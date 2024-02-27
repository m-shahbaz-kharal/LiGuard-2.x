

import open3d as o3d
import open3d.visualization.gui as gui

import os, time
import cv2
import numpy as np
import threading

class CameraCalibrationGUI:
    def get_callbacks_dict():
        return {'start_capture': [],
                'end_capture': [],
                'done': [],
                'cancel': []}
    def __init__(self, app: gui.Application, callbacks = get_callbacks_dict()):
        # app
        self.app = app
        self.app_path = os.path.dirname(os.path.realpath(__file__))
        
        self.mwin = app.create_window("Camera Calibration", 520, 500, x=0, y=30)
        self.em = self.mwin.theme.font_size
        self.callbacks = callbacks
        
        self.__init__layout__()
        self.__init_vars__()
        
    def __init__layout__(self):
        # base container
        self.base_container = gui.ScrollableVert(self.em * 0.2, gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        # help text
        self.help_label = gui.Label(
"""
Instructions:
- Please configure the checkerboard settings and press 'Start Capture' to begin.
- Press the space bar to capture the checkerboard pattern.
- Capture images by varying the position and orientation of the checkerboard.
- Usually 10-20 images are enough for calibration but more images are better.
- Press End Capture to stop capturing images.
- Press 'Calibrate' to calculate the calibration matrix.
- A calibration matrix and reprojection error will be displayed.
- If the reprojection error is too high, you can restart the process.
- Press 'Done' to finish the calibration process.
""")
        self.base_container.add_child(self.help_label)
        # checkerboard settings
        vert = gui.Vert(self.em * 0.2)
        lbl = gui.Label("Checkerboard Settings:")
        
        horiz = gui.Horiz(self.em * 0.2)
        lbl1 = gui.Label("# of Horizontal Corners:")
        self.horz_corners_textedit = gui.TextEdit()
        self.horz_corners_textedit.text_value = "5"
        horiz.add_child(lbl1)
        horiz.add_child(self.horz_corners_textedit)
        vert.add_child(horiz)
        
        horiz = gui.Horiz(self.em * 0.2)
        lbl2 = gui.Label("# of Vertical Corners:")
        self.vert_corners_textedit = gui.TextEdit()
        self.vert_corners_textedit.text_value = "7"
        horiz.add_child(lbl2)
        horiz.add_child(self.vert_corners_textedit)
        vert.add_child(horiz)
        
        self.base_container.add_child(lbl)
        self.base_container.add_child(vert)
        
        # buttons
        horiz = gui.Horiz(self.em * 0.2)
        self.capture_button = gui.Button("Start Capture")
        self.calibrate_button = gui.Button("Calibrate")
        self.done_button = gui.Button("Done")
        horiz.add_child(self.capture_button)
        horiz.add_child(self.calibrate_button)
        horiz.add_child(self.done_button)
        self.base_container.add_child(horiz)
        
        # # of images captured
        horiz = gui.Horiz(self.em * 0.2)
        lbl = gui.Label("# of Images Captured:")
        self.num_images_textedit =  gui.Label("0")
        horiz.add_child(lbl)
        horiz.add_child(self.num_images_textedit)
        self.base_container.add_child(horiz)
        
        # output matrix
        horiz = gui.Horiz(self.em * 0.2)
        lbl = gui.Label("Calibration Matrix:")
        self.calib_matrix_textedit =  gui.Label("")
        self.calib_matrix_textedit.text = "No calibration matrix available."
        self.calib_matrix_textedit.enabled = False
        horiz.add_child(lbl)
        horiz.add_child(self.calib_matrix_textedit)
        self.base_container.add_child(horiz)
        
        # output reprojection error
        horiz = gui.Horiz(self.em * 0.2)
        lbl = gui.Label("Reprojection Error:")
        self.reprojection_error_textedit =  gui.Label("")
        self.reprojection_error_textedit.text = "No reprojection error available."
        self.reprojection_error_textedit.enabled = False
        horiz.add_child(lbl)
        horiz.add_child(self.reprojection_error_textedit)
        self.base_container.add_child(horiz)
        
        # error message
        self.error_label = gui.Label("")
        self.base_container.add_child(self.error_label)
        
        # add to main window
        self.mwin.add_child(self.base_container)
        
    def __init_vars__(self):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # for use by opencv calibrateCamera
        self.generated_patterns = []
        self.captured_patterns = []
        # gui locks
        self.capture_lock = threading.Event()
        # default gui states
        self.horz_corners_textedit.enabled = True
        self.vert_corners_textedit.enabled = True
        self.capture_button.enabled = True
        self.calibrate_button.enabled = False
        self.done_button.enabled = False
        self.calib_matrix_textedit.text = "No calibration matrix available."
        self.reprojection_error_textedit.text = "No reprojection error available."
        # output
        self.calib_success, self.calib_matrix, self.calib_dist, self.calib_rvecs, self.calib_tvecs = None, None, None, None, None
        
    def __set_callbacks__(self):
        self.capture_button.set_on_clicked(self.__start_or_end_capture__)
        self.calibrate_button.set_on_clicked(self.__calibrate_button_callback__)
        self.done_button.set_on_clicked(self.__done_button_callback__)

    def __start_or_end_capture__(self):
        if not self.capture_lock.is_set(): self.capture_lock.set()
        else: self.capture_lock.clear()
        
        if self.capture_lock.is_set(): threading.Thread(target=self.__record_data__).start()
            
            
    def __record_data__(self):
        # clear the generated and captured patterns arrays
        self.generated_patterns = []
        self.captured_patterns = []
        # pattern size
        pattern_size = (int(self.horz_corners_textedit.text_value), int(self.vert_corners_textedit.text_value))
        # prepare the pattern points
        pattern = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        pattern[:,:2] = np.mgrid[0:pattern_size[1], 0:pattern_size[0]].T.reshape(-1,2)
        # init capture
        cap = cv2.VideoCapture(0)
        # capture loop
        while self.capture_lock.is_set():
            # read the frame
            ret, frame = cap.read()
            if not ret:
                self.error_label.text = "Error: Could not capture frame."
                break
            # convert to gray
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.image_shape = gray.shape[::-1]
            # find the corners
            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            # if found, draw the corners
            if found: frame = cv2.drawChessboardCorners(frame, pattern_size, corners, found)
            # if the user pressed space bar and the corners were found, add the pattern to the generated patterns
            if cv2.waitKey(10) & 0xFF == ord(' ') and found:
                self.generated_patterns.append(pattern)
                # refine the corners and add to the captured patterns
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), self.criteria)
                frame = cv2.drawChessboardCorners(frame, pattern_size, corners2, found)
                self.captured_patterns.append(corners2)
            # refresh the gui
            self.__refresh_gui__()
            # display the image
            cv2.imshow('Capture Window', frame)
        # release the capture
        cap.release()
        # destroy the window
        cv2.destroyAllWindows()
        # refresh the gui
        self.__refresh_gui__()
        
    def __calibrate_button_callback__(self):
        self.calib_success, self.calib_matrix, self.calib_dist, self.calib_rvecs, self.calib_tvecs = cv2.calibrateCamera(self.generated_patterns, self.captured_patterns, self.image_shape, None, None)
        mean_error = 0
        for i in range(len(self.generated_patterns)):
            imgpoints2, _ = cv2.projectPoints(self.generated_patterns[i], self.calib_rvecs[i], self.calib_tvecs[i], self.calib_matrix, self.calib_dist)
            error = cv2.norm(self.captured_patterns[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        self.reprojection_error = mean_error/len(self.generated_patterns)
        # refresh the gui
        self.__refresh_gui__()
        
    def __done_button_callback__(self):
        for cb in self.callbacks['done']: cb(self.calib_matrix, self.calib_dist, self.calib_rvecs, self.calib_tvecs)
        
    def __refresh_gui__(self):
        def text_update(): self.num_images_textedit.text = str(len(self.generated_patterns))
        gui.Application.instance.post_to_main_thread(self.mwin, lambda: text_update())
        if self.capture_lock.is_set():
            self.horz_corners_textedit.enabled = False
            self.vert_corners_textedit.enabled = False
            self.capture_button.text = "End Capture"
            if len(self.captured_patterns) < 10: self.capture_button.enabled = False
            else: self.capture_button.enabled = True
            self.calibrate_button.enabled = False
            self.done_button.enabled = False
            self.calib_matrix_textedit.text = "No calibration matrix available."
            self.reprojection_error_textedit.text = "No reprojection error available."
            self.error_label.text = ""
        else:
            self.horz_corners_textedit.enabled = True
            self.vert_corners_textedit.enabled = True
            self.capture_button.text = "Start Capture"
            self.capture_button.enabled = True
            if len(self.captured_patterns) >= 10: self.calibrate_button.enabled = True
            else: self.calibrate_button.enabled = False
            if self.calib_success != None:
                self.done_button.enabled = True
                self.calib_matrix_textedit.enabled = True
                self.calib_matrix_textedit.text = str(self.calib_matrix)
                self.reprojection_error_textedit.enabled = True
                self.reprojection_error_textedit.text = str(self.reprojection_error)
                self.error_label.text = ""
            else: self.done_button.enabled = False
        
def main():
    app = gui.Application.instance
    app.initialize()
    
    calib_callbacks = CameraCalibrationGUI.get_callbacks_dict()
    calib_callbacks['done'] = [lambda calib_matrix, calib_dist, calib_rvecs, calib_tvecs: print(f"Calibration Matrix: {calib_matrix}\nDistortion Coefficients: {calib_dist}\nRotation Vectors: {calib_rvecs}\nTranslation Vectors: {calib_tvecs}")]
    calib_gui = CameraCalibrationGUI(app, calib_callbacks)
    calib_gui.__set_callbacks__()
    
    app.run()

main()
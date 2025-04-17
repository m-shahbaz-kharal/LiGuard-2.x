import open3d.visualization.gui as gui

import os
import platform
isMacOS = (platform.system() == "Darwin")
from liguard.gui.gui_utils import resolve_for_application_root, resolve_for_default_workspace
import time
import yaml

import inspect

class Logger:
    # Logging levels
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4
    # Logging level strings and colors
    __level_string__ = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
    __level_color__ = [gui.Color(255,255,255,255), gui.Color(0,255,0,255), gui.Color(255,255,0,255), gui.Color(255,0,0,255), gui.Color(255,0,255,255)]
    
    def __init__(self, app: gui.Application=None):
            """
            Initializes the LoggerGUI object.

            Args:
                app (gui.Application): The application object.
            """
            self.app = app

            if self.app:
                # create the logger window
                if isMacOS: self.mwin = app.create_window("Logs", 440, 1080) # macOS has a bug with window position
                else: self.mwin = app.create_window("Logs", 440, 1080, x=1480, y=30)
                self.mwin.set_on_close(lambda: False)
                
                # set layout
                self.em = self.mwin.theme.font_size
                self.__init__layout__()
        
    def reset(self, cfg: dict=None):
        """
        Resets the logger configuration based on the provided dictionary.

        Args:
            cfg (dict): The dictionary containing the logger configuration.
        """
        if cfg:
            path = cfg['logging']['logs_dir']
            level = cfg['logging']['level']
        else:
            path = None
            level = Logger.DEBUG

        if path:
            if not os.path.isabs(path): path = os.path.join(cfg['data']['pipeline_dir'], path)
            if not os.path.exists(path): os.makedirs(path, exist_ok=True)
            self.log_file_path = os.path.join(path, time.strftime("log_%Y%m%d-%H%M%S") + ".txt")
        else:
            path = os.path.join(resolve_for_application_root(), 'logs')
            if not os.path.exists(path): os.makedirs(path, exist_ok=True)
            self.log_file_path = os.path.join(path, time.strftime("log_%Y%m%d-%H%M%S") + ".txt")

        if level < Logger.DEBUG or level > Logger.CRITICAL:
            level = Logger.DEBUG
            self.log(f'[gui->logger_gui.py->Logger]: Invalid logging level. Setting to default level: {Logger.__level_string__[level]}', Logger.INFO)
        self.level = level

        self.log('\n\nConfiguartion:\n\n' + yaml.dump(cfg) + '\n\nLog:\n\n', Logger.DEBUG)
        self.__clear_log__()

    def change_level(self, level:int):
            """
            Change the logging level of the logger.

            Args:
                level (int): The new logging level.
            """
            if level < Logger.DEBUG: level = Logger.DEBUG
            elif level > Logger.CRITICAL: level = Logger.CRITICAL
            self.level = level
            self.log(f'[gui->logger_gui.py->Logger]: Logging level changed to {Logger.__level_string__[level]}', Logger.WARNING)

    def log(self, message:str, level:int):
        """
        Logs a message with the specified level.

        Args:
            message (str): The message to be logged.
            level (int): The level of the log message.
        """
        stack = inspect.stack()
        message = \
f"""
{os.path.basename(stack[2].filename)} [Line: {stack[2].lineno}] -> {stack[2].code_context[0].strip()}
|
{stack[1].function}
|
Message:
{message}
"""

        # Write to log file
        txt = f'{time.strftime("%Y-%m-%d %H:%M:%S")} [{Logger.__level_string__[level]}] {message}\n'
        with open(self.log_file_path, 'a') as log_file: log_file.write(txt)
        
        if level < self.level: return

        if hasattr(self, 'mwin') == False:
            print(txt)
            return
        
        def _gui_update():
            # update log container if level is greater than or equal to the current level
            horiz = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
            icon_str = gui.Label(Logger.__level_string__[level])
            icon_str.text_color = Logger.__level_color__[level]
            msg_str = gui.Label(message)
            horiz.add_child(gui.Label('--' * 40))
            horiz.add_child(icon_str)
            horiz.add_child(msg_str)
            horiz.add_child(gui.Label('--' * 40))
            self.log_container.add_child(horiz)
        
            # request redraw
            self.mwin.set_needs_layout()
            self.mwin.post_redraw()

        self.app.post_to_main_thread(self.mwin, _gui_update)
        
    def __init__layout__(self):
        # Create the layout for the logger window
        self.base_container = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        self.log_container = gui.WidgetProxy()
        
        # Create a scrollable container for the log messages
        scroll_vert = gui.ScrollableVert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.log_container.set_widget(scroll_vert)
        self.button_container = gui.Horiz(self.em * 0.2, gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        
        # Create a clear button
        clear_button = gui.Button("Clear")
        clear_button.set_on_clicked(self.__clear_log__)
        self.button_container.add_child(clear_button)
        self.button_container.add_stretch()

        # Create a status strip
        self.status_container = gui.Horiz(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.status_current_frame_idx = gui.Label("Frame Index: NA")
        self.status_container.add_stretch()
        self.status_container.add_child(self.status_current_frame_idx)
        
        # Add the containers to the base container
        self.base_container.add_child(self.button_container)
        self.base_container.add_child(self.status_container)
        self.base_container.add_child(self.log_container)
        
        # Add the base container to the main window
        self.mwin.add_child(self.base_container)
        
    def __clear_log__(self):
        if hasattr(self, 'mwin') == False: return
        # Clear the log messages by recreeating the log container
        scroll_vert = gui.ScrollableVert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.log_container.set_widget(scroll_vert)

    def gui_set_frame(self, idx: int):
        self.app.post_to_main_thread(self.mwin, lambda: self._set_frame(idx))

    def _set_frame(self, idx: int):
        if hasattr(self, 'mwin'): self.status_current_frame_idx.text = f'Frame Index: {idx}'

    def quit(self):
        if hasattr(self, 'mwin') == False: return
        del self.mwin
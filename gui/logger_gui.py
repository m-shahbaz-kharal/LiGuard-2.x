import open3d.visualization.gui as gui

import os
import time
import yaml

class Logger:
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4
    __level_string__ = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
    __level_color__ = [gui.Color(255,255,255,255), gui.Color(0,255,0,255), gui.Color(255,255,0,255), gui.Color(255,0,0,255), gui.Color(255,0,255,255)]
    
    def __init__(self, app: gui.Application):
        self.app = app

        self.mwin = app.create_window("Log", 440, 1080, x=1480, y=30)
        self.em = self.mwin.theme.font_size

        self.mwin.set_on_close(lambda: False)

        self.__init__layout__()
        self.mwin.post_redraw()
        
    def reset(self, cfg: dict):
        level = cfg['logging']['level']
        path = cfg['logging']['path']
        
        if not os.path.exists(path):
            if '.txt' in path: os.makedirs(os.path.dirname(path), exist_ok=True)
            else: os.makedirs(path, exist_ok=True)
        
        if os.path.isdir(path): self.log_file_path = os.path.join(path, time.strftime("log_%Y%m%d-%H%M%S") + ".txt")
        else: self.log_file_path = path

        if level < Logger.DEBUG or level > Logger.CRITICAL:
            level = Logger.DEBUG
            self.log(f'[gui->logger_gui.py->Logger]: Invalid logging level. Setting to default level: {Logger.__level_string__[level]}', Logger.INFO)
        self.level = level

        self.log('\n\nConfiguartion:\n\n' + yaml.dump(cfg) + '\n\nLog:\n\n', Logger.DEBUG)
        self.__clear_log__()

    def change_level(self, level:int):
        self.level = level
        self.log(f'[gui->logger_gui.py->Logger]: Logging level changed to {Logger.__level_string__[level]}', Logger.INFO)

    def log(self, message:str, level:int):
        txt = f'{time.strftime("%Y-%m-%d %H:%M:%S")} [{Logger.__level_string__[level]}] {message}\n'
        with open(self.log_file_path, 'a') as log_file: log_file.write(txt)

        if level < self.level: return
        
        horiz = gui.Horiz(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        icon_str = gui.Label(Logger.__level_string__[level])
        icon_str.text_color = Logger.__level_color__[level]
        msg_str = gui.Label(message)
        horiz.add_child(icon_str)
        horiz.add_child(msg_str)
        self.log_container.add_child(horiz)
        self.mwin.set_needs_layout()
        self.mwin.post_redraw()
        
    def __init__layout__(self):
        self.base_container = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        self.log_container = gui.WidgetProxy()
        
        scroll_vert = gui.ScrollableVert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.log_container.set_widget(scroll_vert)
        self.button_container = gui.Horiz(self.em * 0.2, gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        
        clear_button = gui.Button("Clear")
        clear_button.set_on_clicked(self.__clear_log__)
        self.button_container.add_child(clear_button)
        self.button_container.add_stretch()

        self.base_container.add_child(self.button_container)
        self.base_container.add_child(self.log_container)
        
        self.mwin.add_child(self.base_container)
        
    def __clear_log__(self):
        scroll_vert = gui.ScrollableVert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.log_container.set_widget(scroll_vert)

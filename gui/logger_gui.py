import open3d.visualization.gui as gui

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

        self.__init__layout__()
        self.mwin.post_redraw()
        
    def reset(self, cfg: dict, level:int = INFO):
        if level < Logger.DEBUG or level > Logger.CRITICAL: raise ValueError("Invalid log level")
        self.level = level
        
        self.log_file_path = time.strftime("log_%Y%m%d-%H%M%S") + ".txt"

        self.log('\n\nConfiguartion:\n\n' + yaml.dump(cfg) + '\n\nLog:\n\n', Logger.CRITICAL)
        self.__clear_log__()

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

if __name__ == "__main__":
    app = gui.Application.instance
    app.initialize()
    
    logger = Logger(app)
    
    logger.log("This is a debug message", Logger.DEBUG)
    logger.log("This is an info message", Logger.INFO)
    logger.log("This is a warning message", Logger.WARNING)
    logger.log("This is an error message", Logger.ERROR)
    logger.log("This is a critical message", Logger.CRITICAL)

    import keyboard
    def clear_and_test_again():
        logger.log("Clearing log", Logger.INFO)
        logger.log("This is a debug message", Logger.DEBUG)
        logger.log("This is an info message", Logger.INFO)
        logger.log("This is a warning message", Logger.WARNING)
        logger.log("This is an error message", Logger.ERROR)
        logger.log("This is a critical message", Logger.CRITICAL)

    keyboard.add_hotkey('c', clear_and_test_again)
    
    app.run()
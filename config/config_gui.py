import open3d as o3d
import open3d.visualization.gui as gui

import os, time
import yaml
from easydict import EasyDict
import ast

class BaseConfiguration:
    def get_callbacks_dict():
        return {'new_config': [],
                'open_config': [],
                'apply_and_save_config': [],
                'apply_and_save_as_config': [],
                'apply_only_config': [],
                'quit_config': []}
        
    def __init__(self, app: gui.Application, callbacks = get_callbacks_dict()):
        self.app = app
        self.app_path = os.path.dirname(os.path.realpath(__file__))
        
        self.mwin = app.create_window("Configuration", 480, 1080, x=0, y=30)
        self.em = self.mwin.theme.font_size
        self.callbacks = callbacks
        self.mwin.set_on_close(self.__quit_config__)
        
        self.__init__layout__()
        
    def __init__layout__(self):
        # base container
        self.base_container = gui.ScrollableVert(self.em * 0.2, gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        
        # file_io controls
        # -- define text edit
        self.config_file_path_textedit =  gui.TextEdit()
        self.config_file_path_textedit.text_value = "None"
        self.config_file_path_textedit.enabled = False
        #   add to file_io container
        self.base_container.add_child(self.config_file_path_textedit)
        # buttons for new, open, save, save as, and apply
        # -- define buttons
        button_container = gui.Horiz(self.em * 0.2)
        self.new_config_button = gui.Button("New")
        self.open_config_button = gui.Button("Open")
        self.apply_and_save_config_button = gui.Button("Apply & Save")
        self.apply_and_save_as_config_button = gui.Button("Apply & Save As")
        self.apply_only_config_button = gui.Button("Apply Only")
        # -- set callbacks
        self.new_config_button.set_on_clicked(self.__new_config__)
        self.open_config_button.set_on_clicked(self.__open_config__)
        self.apply_and_save_config_button.set_on_clicked(self.__apply_and_apply_and_save_config__)
        self.apply_and_save_as_config_button.set_on_clicked(self.__apply_and_apply_and_save_as_config__)
        self.apply_only_config_button.set_on_clicked(self.__apply_only_config__)
        # -- add to button container
        button_container.add_stretch()
        button_container.add_child(self.new_config_button)
        button_container.add_child(self.open_config_button)
        button_container.add_child(self.apply_and_save_config_button)
        button_container.add_child(self.apply_and_save_as_config_button)
        button_container.add_child(self.apply_only_config_button)
        # add to base container
        self.base_container.add_child(button_container)
        
        # all the generated configuration gui will be set here
        self.generated_config = gui.WidgetProxy()
        self.generated_config_gui_dict = EasyDict()
        # add to base container
        self.base_container.add_child(self.generated_config)
        
        # add to main window
        self.mwin.add_child(self.base_container)
        
    def set_callbacks(self, callbacks):
        self.callbacks = callbacks
        
    def load_config(self, cfg_path):
        with open(cfg_path) as f: cfg = yaml.safe_load(f)
        cfg = EasyDict(cfg)
        return cfg
    
    def apply_and_save_config(self, cfg, cfg_path):    
        def __convert_to_dict__(item):
            if isinstance(item, EasyDict):
                item = dict(item)
                for key in item: item[key] = __convert_to_dict__(item[key])
            return item
    
        with open(cfg_path, 'w') as f:
            cfg = __convert_to_dict__(cfg)
            yaml.dump(cfg, f, default_flow_style=False)
            
    def generate_config_gui_from_cfg(self, item, container, parent_keys=[], margin = 0.2):
        G = self.generated_config_gui_dict
        if type(item) == EasyDict:
            for key in item.keys():
                label_text = ' '.join([k.capitalize() for k in key.split('_')])
                if type(item[key]) == EasyDict:
                    label_text = key[0].upper() + key[1:]
                    collapsable_container = gui.CollapsableVert(label_text, self.em * 0.2, gui.Margins(self.em * margin, self.em * 0.2, self.em * 0.2, self.em * 0.2))
                    self.generate_config_gui_from_cfg(item[key], collapsable_container, parent_keys + [key], margin + 0.2)
                    container.add_child(collapsable_container)
                elif type(item[key]) in [str, int, float]:
                    sub_container = gui.Horiz()
                    sub_container.add_child(gui.Label(label_text + ":"))
                    global_key = ".".join(parent_keys + [key])
                    G[global_key] = {'view': gui.TextEdit(), 'type': type(item[key])}
                    G[global_key].view.text_value = str(item[key])
                    sub_container.add_child(G[global_key].view)
                    container.add_child(sub_container)
                elif type(item[key]) == list:
                    sub_container = gui.Horiz()
                    sub_container.add_child(gui.Label(label_text + ":"))
                    global_key = ".".join(parent_keys + [key])
                    G[global_key] = {'view': gui.TextEdit(), 'type': type(item[key][0])}
                    G[global_key].view.text_value = str(item[key])
                    sub_container.add_child(G[global_key].view)
                    container.add_child(sub_container)
                elif type(item[key]) == bool:
                    sub_container = gui.Horiz()
                    sub_container.add_child(gui.Label(label_text + ":"))
                    global_key = ".".join(parent_keys + [key])
                    G[global_key] = {'view': gui.CheckBox(), 'type': type(item[key])}
                    G[global_key].view.checked = item[key]
                    sub_container.add_child(G[global_key].view)
                    container.add_child(sub_container)
                else:
                    raise Exception("Unsupported type: {}".format(type(item[key])))
                
    def update_cfg_from_gui(self, item, parent_keys=[]):
        G = self.generated_config_gui_dict
        if type(item) == EasyDict:
            for key in item.keys():
                global_key = ".".join(parent_keys + [key])
                if type(item[key]) == EasyDict:
                    self.update_cfg_from_gui(item[key], parent_keys + [key])
                elif type(item[key]) in [str, int, float]:
                    item[key] = G[global_key].type(G[global_key].view.text_value)
                elif type(item[key]) == list:
                    item[key] = ast.literal_eval(G[global_key].view.text_value)
                    item[key] = [G[global_key].type(i) for i in item[key]]    
                elif type(item[key]) == bool:
                    global_key = ".".join(parent_keys + [key])
                    item[key] = G[global_key].type(G[global_key].view.checked)
                else:
                    raise Exception("Unsupported type: {}".format(type(item[key])))
                
    def __show_issue_dialog__(self):
        dialog = gui.Dialog("Configuration GUI")
        horiz = gui.Horiz(0, gui.Margins(self.em * 0.6, self.em * 0.6, self.em * 0.6, self.em * 0.4))
        horiz.add_child(gui.Label(self.issue_text))
        dialog.add_child(horiz)
        self.mwin.show_dialog(dialog)
        
    def __close_issue_dialog__(self):
        self.mwin.close_dialog()
        
    def __new_config__(self):
        self.cfg = self.load_config(os.path.join(self.app_path, 'config_template.yml'))
        self.config_file_path_textedit.text_value = os.path.join(self.app_path, time.strftime("%Y%m%d-%H%M%S") + ".yml")
        cfg_gui = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.generate_config_gui_from_cfg(self.cfg, cfg_gui, ['cfg'])
        self.generated_config.set_widget(cfg_gui)
        
        for callback in self.callbacks['new_config']: callback(self.cfg)
            
    def __open_config__(self):
        def load_cfg_and_close_dialog(cfg_path):
            self.config_file_path_textedit.text_value = cfg_path
            self.mwin.close_dialog()
            
            self.cfg = self.load_config(cfg_path)
            cfg_gui = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
            self.generate_config_gui_from_cfg(self.cfg, cfg_gui, ['cfg'])
            self.generated_config.set_widget(cfg_gui)

        read_config_file_dialog = gui.FileDialog(gui.FileDialog.OPEN, "Load Configuration", self.mwin.theme)
        read_config_file_dialog.add_filter(".yml", "LiGuard Configuration (.yml)")
        read_config_file_dialog.set_on_cancel(lambda: self.mwin.close_dialog())
        read_config_file_dialog.set_on_done(lambda path: load_cfg_and_close_dialog(path))
        self.mwin.show_dialog(read_config_file_dialog)
        
        for callback in self.callbacks['open_config']: callback(self.cfg)
    
    def __apply_and_apply_and_save_config__(self):
        try:
            self.update_cfg_from_gui(self.cfg, ['cfg'])
            self.apply_and_save_config(self.cfg, self.config_file_path_textedit.text_value)
        except:
            self.issue_text = "Failed to save configuration file."
            self.__show_issue_dialog__()
            time.sleep(2)
            self.__close_issue_dialog__()
        for callback in self.callbacks['apply_and_save_config']: callback(self.cfg)
            
    def __apply_and_apply_and_save_as_config__(self):
        def save_cfg_and_close_dialog(cfg, cfg_path):
            self.config_file_path_textedit.text_value = cfg_path
            self.update_cfg_from_gui(self.cfg, ['cfg'])
            self.apply_and_save_config(cfg, cfg_path)
            self.mwin.close_dialog()
            
        if hasattr(self, 'cfg'):
            apply_and_save_config_file_dialog = gui.FileDialog(gui.FileDialog.SAVE, "Save Configuration", self.mwin.theme)
            apply_and_save_config_file_dialog.add_filter(".yml", "LiGuard Configuration (.yml)")
            apply_and_save_config_file_dialog.set_on_cancel(lambda: self.mwin.close_dialog())
            apply_and_save_config_file_dialog.set_on_done(lambda path: save_cfg_and_close_dialog(self.cfg, path))
            self.mwin.show_dialog(apply_and_save_config_file_dialog)
        
        for callback in self.callbacks['apply_and_save_as_config']: callback(self.cfg)
            
    def __apply_only_config__(self):
        self.update_cfg_from_gui(self.cfg, ['cfg'])
        for callback in self.callbacks['apply_only_config']: callback(self.cfg)
        
    def __quit_config__(self):
        print("Quitting...")
        for callback in self.callbacks['quit_config']: callback(self.cfg)
        return True
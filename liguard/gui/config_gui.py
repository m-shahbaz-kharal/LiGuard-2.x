import open3d.visualization.gui as gui

import os
import sys
import time
import yaml
import ast

user_home_dir = os.path.expanduser("~")
application_root_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
def get_abs_path(path:str) -> str:
    if not os.path.isabs(path): path = os.path.join(application_root_dir, path)
    return os.path.abspath(path)

class BaseConfiguration:
    def get_callbacks_dict():
        """
        Generates a template dictionary containing keys for applicable callback function names to be used as `BaseConfiguration` class's `callbacks` argument.

        Returns:
            dict: A dictionary with keys representing applicable callback actions and values as empty lists.
        """
        return {
            'new_config': [],       # List to store callback functions for 'new_config' action
            'open_config': [],      # List to store callback functions for 'open_config' action
            'save_config': [],      # List to store callback functions for 'save_config' action
            'save_as_config': [],   # List to store callback functions for 'save_as_config' action
            'apply_config': [],     # List to store callback functions for 'apply_config' action
            'quit_config': []       # List to store callback functions for 'quit_config' action
        }
        
    def __init__(self, app: gui.Application, callbacks = get_callbacks_dict()):
        """
        Initializes the ConfigGUI class.

        Args:
            app (gui.Application): The application object.
            callbacks (dict): A dictionary containing the callbacks for the GUI.
        """
        self.app = app
        self.callbacks = callbacks

        # Create a window for the configuration GUI
        self.mwin = app.create_window("Configuration", 480, 1080, x=0, y=30)
        # Define the em unit
        self.em = self.mwin.theme.font_size
        # Set the close callback
        self.mwin.set_on_close(self.__quit_config__)
        
        # Initialize the layout
        self.__init__layout__()
        
    def __init__layout__(self):
        # top container
        self.top_container = gui.Vert(self.em * 0.2)

        # file_io controls
        # -- define text edit
        self.config_file_path_textedit =  gui.TextEdit()
        self.config_file_path_textedit.text_value = "None"
        self.config_file_path_textedit.enabled = False
        #   add to file_io container
        self.top_container.add_child(self.config_file_path_textedit)
        # buttons for new, open, save, save as, and apply
        # -- define buttons
        button_container = gui.Horiz(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.new_config_button = gui.Button("New")
        self.open_config_button = gui.Button("Open")
        self.save_config_button = gui.Button("Save")
        self.save_as_config_button = gui.Button("Save As")
        self.apply_config_button = gui.Button("Apply")
        # -- set callbacks
        self.new_config_button.set_on_clicked(self.__new_config__)
        self.open_config_button.set_on_clicked(self.__open_config__)
        self.save_config_button.set_on_clicked(self.__save_config__)
        self.save_as_config_button.set_on_clicked(self.__save_as_config__)
        self.apply_config_button.set_on_clicked(self.__apply_config__)
        # -- add to button container
        button_container.add_stretch()
        button_container.add_child(self.new_config_button)
        button_container.add_child(self.open_config_button)
        button_container.add_child(self.save_config_button)
        button_container.add_child(self.save_as_config_button)
        button_container.add_child(self.apply_config_button)
        
        # add to top container
        self.top_container.add_child(button_container)

        # base container
        self.base_container = gui.ScrollableVert(self.em * 0.2, gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        
        # all the generated configuration gui will be set here
        self.generated_config = gui.WidgetProxy()
        self.generated_config_gui_dict = dict()
        
        # add to base container
        self.base_container.add_child(self.generated_config)
        self.base_container.add_stretch()

        # add to top container
        self.top_container.add_child(self.base_container)
        
        # add to main window
        self.mwin.add_child(self.top_container)
        
    def update_callbacks(self, callbacks):
        """
        Update the callbacks for the GUI.

        Parameters:
        callbacks (dict): A dictionary containing the callbacks for the GUI.

        Returns:
        None
        """
        self.callbacks = callbacks
        
    def load_config(self, cfg_path):
        """
        Load a configuration file.

        Args:
            cfg_path (str): The path to the configuration file.

        Returns:
            dict: The loaded configuration as a dictionary.
        """
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        return cfg
    
    def save_config(self, cfg, cfg_path):
        """
        Save the configuration to a YAML file.

        Args:
            cfg (dict): The configuration dictionary to be saved.
            cfg_path (str): The path to the YAML file.

        Returns:
            None
        """
        with open(cfg_path, 'w') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
            
    def __update_gui_from_cfg__(self, item, container, parent_keys=[], margin=0.2):
        """
        This function recursively generates the GUI from the configuration dictionary based on the item types.
        """
        G = self.generated_config_gui_dict
        if type(item) == dict:
            for key in item.keys():
                label_text = key
                if type(item[key]) == dict:
                    collapsable_container = gui.CollapsableVert(label_text, self.em * 0.2, gui.Margins(self.em * margin, self.em * 0.2, self.em * 0.2, self.em * 0.2))
                    collapsable_container.set_is_open(False)
                    self.__update_gui_from_cfg__(item[key], collapsable_container, parent_keys + [key], margin + 0.2)
                    container.add_child(collapsable_container)
                elif type(item[key]) in [str, int, float]:
                    sub_container = gui.Horiz()
                    sub_container.add_child(gui.Label(label_text + ":"))
                    global_key = ".".join(parent_keys + [key])
                    G[global_key] = {'view': gui.TextEdit(), 'type': type(item[key])}
                    G[global_key]['view'].text_value = str(item[key])
                    sub_container.add_child(G[global_key]['view'])
                    if key.endswith('_dir') or key.endswith('_file'):
                        browse_button = gui.Button("...")
                        def create_browse_dialog(title=label_text, path_edit=G[global_key]['view']):
                            def on_done(path):
                                path_edit.text_value = path
                                self.__close_dialog__()
                            dialog_title = f"Select {title.upper()}"
                            mode = gui.FileDialog.OPEN_DIR if title.endswith('_dir') else gui.FileDialog.OPEN
                            self.__show_path_dialog__(dialog_title, mode, self.last_workspace_dir, "", on_done=on_done)
                        browse_button.set_on_clicked(create_browse_dialog)
                        sub_container.add_child(browse_button)
                    sub_container.add_stretch()
                    container.add_child(sub_container)
                elif type(item[key]) == list:
                    sub_container = gui.Horiz()
                    sub_container.add_child(gui.Label(label_text + ":"))
                    global_key = ".".join(parent_keys + [key])
                    G[global_key] = {'view': gui.TextEdit(), 'type': type(item[key][0])}
                    G[global_key]['view'].text_value = str(item[key])
                    sub_container.add_child(G[global_key]['view'])
                    container.add_child(sub_container)
                elif type(item[key]) == bool:
                    sub_container = gui.Horiz()
                    sub_container.add_child(gui.Label(label_text + ":"))
                    global_key = ".".join(parent_keys + [key])
                    G[global_key] = {'view': gui.Checkbox(key), 'type': type(item[key])}
                    G[global_key]['view'].checked = item[key]
                    sub_container.add_child(G[global_key]['view'])
                    container.add_child(sub_container)
                else:
                    raise Exception("Unsupported type: {}".format(type(item[key])))
                
    def __update_cfg_from_gui__(self, item, parent_keys=[]):
        G = self.generated_config_gui_dict
        # Check if the item is a dictionary
        if type(item) == dict:
            # Iterate over all keys in the dictionary
            for key in item.keys():
                # Create a global key by joining the parent keys and the current key
                global_key = ".".join(parent_keys + [key])
                # If the value of the current key is a dictionary, recursively call the function
                if type(item[key]) == dict:
                    self.__update_cfg_from_gui__(item[key], parent_keys + [key])
                # If the value of the current key is a string, integer or float
                elif global_key in G:
                    if type(item[key]) in [str, int, float]:
                        # Update the value of the key in the item dictionary with the value from the GUI
                        item[key] = G[global_key]['type'](G[global_key]['view'].text_value)
                    # If the value of the current key is a list
                    elif type(item[key]) == list:
                        # Parse the list from the string value in the GUI
                        item[key] = ast.literal_eval(G[global_key]['view'].text_value)
                        # Convert each item in the list to the appropriate type
                        item[key] = [G[global_key]['type'](i) for i in item[key]]    
                    # If the value of the current key is a boolean
                    elif type(item[key]) == bool:
                        # Update the value of the key in the item dictionary with the checked state from the GUI
                        item[key] = G[global_key]['type'](G[global_key]['view'].checked)
                    # If the value of the current key is of an unsupported type
                    else:
                        # Raise an exception
                        raise Exception("Unsupported type: {}".format(type(item[key])))
                    
    def __show_dialog__(self, dialog):
        if not hasattr(self, 'showing_dialog'): self.showing_dialog = True
        elif self.showing_dialog: self.__close_dialog__()
        self.showing_dialog = True
        self.mwin.show_dialog(dialog)
        
    def __close_dialog__(self):
        self.showing_dialog = False
        self.mwin.close_dialog()
                
    def __show_issue_dialog__(self, issue_text):
        # create a dialog
        dialog = gui.Dialog("Configuration GUI")
        vert = gui.Vert(0, gui.Margins(self.em * 0.6, self.em * 0.6, self.em * 0.6, self.em * 0.4))
        
        msg_label = gui.Label(f'Error: {issue_text}')
        vert.add_child(msg_label)
        
        ok_button = gui.Button("OK")
        vert.add_child(ok_button)
        ok_button.set_on_clicked(lambda: self.__close_dialog__())
        
        # show the dialog
        dialog.add_child(vert)
        self.__show_dialog__(dialog)

    def show_input_dialog(self, title, query, key, ans_base_type=gui.NumberEdit, ans_type=gui.NumberEdit.INT, value_variable='int_value'):
        # Create a dialog and base layout
        dialog = gui.Dialog(title)
        layout = gui.Vert(0.25 * self.em, gui.Margins(1 * self.em))

        # First row containing Label and Input widget
        h_layout = gui.Horiz(0, gui.Margins(self.em * 0.6, self.em * 0.6, self.em * 0.6, self.em * 0.4))
        
        label = gui.Label(query)
        h_layout.add_child(label)
        self.input_dialog_key = key
        self.input_dialog_widget = ans_base_type(ans_type)
        self.input_dialog_widget_value_variable = value_variable
        h_layout.add_child(self.input_dialog_widget)
        layout.add_child(h_layout)
        
        # Second row containing OK button
        ok_button = gui.Button("OK")
        def ok_button_callback():
            self.cfg[self.input_dialog_key] = getattr(self.input_dialog_widget, self.input_dialog_widget_value_variable)
            self.__close_dialog__()
        ok_button.set_on_clicked(ok_button_callback)
        layout.add_child(ok_button)

        # Add the layout to the dialog and show the dialog
        dialog.add_child(layout)
        self.__show_dialog__(dialog)

    def get_input_dialog_value(self, key):
        if hasattr(self, 'cfg') and key in self.cfg: return self.cfg.pop(key)
        return None
    
    def __show_path_dialog__(self, title: str, mode: gui.FileDialog.Mode, initial_path: str, filter_type: str, filter_name: str = '', on_done = lambda path: print(path)):
        path_dialog = gui.FileDialog(mode, title, self.mwin.theme)
        if filter_type != "": path_dialog.add_filter(filter_type, filter_name)
        path_dialog.set_on_cancel(lambda: self.__close_dialog__())
        path_dialog.set_on_done(on_done)
        if os.path.exists(initial_path): path_dialog.set_path(initial_path)
        self.__show_dialog__(path_dialog)
        
    def __new_config__(self):
        # Load the default configuration template and set a default configuration file name to the current timestamp
        self.cfg = self.load_config(os.path.join(application_root_dir, 'resources', 'config_template.yml'))
        self.last_workspace_dir = os.path.join(user_home_dir, 'liguard-default-workspace')
        time_stamp = time.strftime("%Y%m%d-%H%M%S")
        self.config_file_path_textedit.text_value = os.path.join(self.last_workspace_dir, f"{time_stamp}.yml")

        # Generate the configuration GUI from the configuration dictionary
        cfg_gui = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
        self.__update_gui_from_cfg__(self.cfg, cfg_gui, ['cfg'])
        self.generated_config.set_widget(cfg_gui)
        
        # Call the callback functions for the 'new_config' action
        for callback in self.callbacks['new_config']: callback(self.cfg)

    def __open_config__(self):
        # Define the function to load the configuration file, update GUI, and close the dialog
        def load_cfg_and_close_dialog(cfg_path):
            self.config_file_path_textedit.text_value = cfg_path
            self.last_workspace_dir = os.path.dirname(cfg_path)
            self.__close_dialog__()
            try:
                self.cfg = self.load_config(cfg_path)
                cfg_gui = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
                self.__update_gui_from_cfg__(self.cfg, cfg_gui, ['cfg'])
                self.generated_config.set_widget(cfg_gui)
                # Call the callback functions for the 'open_config' action
                for callback in self.callbacks['open_config']: callback(self.cfg)
            except:
                self.__show_issue_dialog__('Failed to load configuration file.')

        # make sure the last workspace directory exists
        if not hasattr(self, 'last_workspace_dir'):
            self.last_workspace_dir = os.path.join(user_home_dir, 'liguard-default-workspace')

        # Open the configuration file by asking the user for the path
        self.__show_path_dialog__("Open Configuration", gui.FileDialog.OPEN, self.last_workspace_dir, ".yml", "LiGuard Configuration (.yml)", load_cfg_and_close_dialog)
    
    def __save_config__(self):
        # Update the configuration from the GUI and save it to the specified path
        try:
            self.__update_cfg_from_gui__(self.cfg, ['cfg'])
            if not os.path.exists(self.last_workspace_dir): os.makedirs(self.last_workspace_dir)
            self.save_config(self.cfg, self.config_file_path_textedit.text_value)
            # Call the callback functions for the 'save_config' action
            for callback in self.callbacks['save_config']: callback(self.cfg)
        except:
            self.__show_issue_dialog__('Failed to save configuration file.')
            
    def __save_as_config__(self):
        # Define the function to get configuration from GUI, save to the specified path, and close the dialog
        def save_cfg_and_close_dialog(cfg, cfg_path):
            self.config_file_path_textedit.text_value = cfg_path
            self.last_workspace_dir = os.path.dirname(cfg_path)
            self.__update_cfg_from_gui__(self.cfg, ['cfg'])
            self.save_config(cfg, cfg_path)
            self.__close_dialog__()
            # Call the callback functions for the 'save_as_config' action
            for callback in self.callbacks['save_as_config']: callback(self.cfg)

        # If the configuration exists
        if hasattr(self, 'cfg'):
            if not hasattr(self, 'last_workspace_dir'):
                if not os.path.exists(self.last_workspace_dir): os.makedirs(self.last_workspace_dir)
            # Save the configuration by asking the user for the path
            self.__show_path_dialog__("Save Configuration", gui.FileDialog.SAVE, self.last_workspace_dir, ".yml", "LiGuard Configuration (.yml)", lambda cfg_path: save_cfg_and_close_dialog(self.cfg, cfg_path))
        else:
            self.__show_issue_dialog__('No configuration to save.')
            
    def __apply_config__(self):
        # If the configuration exists
        if hasattr(self, 'cfg'):
            # Update the configuration from the GUI
            self.__update_cfg_from_gui__(self.cfg, ['cfg'])
        
            # Call the callback functions for the 'apply_config' action
            for callback in self.callbacks['apply_config']: callback(self.cfg)
        else:
            self.__show_issue_dialog__('No configuration to apply.')
        
    def __quit_config__(self):
        print("Quitting...")

        # If the configuration exists
        if hasattr(self, 'cfg'):
            # Call the callback functions for the 'quit_config' action
            for callback in self.callbacks['quit_config']: callback(self.cfg)
        else:
            # Still call the callback functions for the 'quit_config' action to signal exit
            for callback in self.callbacks['quit_config']: callback(None)

        # Required by gui.Window.set_on_close
        return True
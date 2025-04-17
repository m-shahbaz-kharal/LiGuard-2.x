import open3d.visualization.gui as gui

from liguard.gui.logger_gui import Logger
from liguard.gui.gui_utils import resolve_for_application_root, resolve_for_default_workspace

import os
import platform
isMacOS = (platform.system() == "Darwin")
import sys
import yaml
import ast

import threading

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
            'apply_config': [],     # List to store callback functions for 'apply_config' action
            'quit_config': []       # List to store callback functions for 'quit_config' action
        }
        
    def __init__(self, app: gui.Application, logger: Logger=None):
        """
        Initializes the ConfigGUI class.

        Args:
            app (gui.Application): The application object.
            logger (Logger): The logger object.
        """
        # Set the application and logger objects
        self.app = app
        if logger: self.log = logger.log
        else: self.log = lambda msg,lvl: print(f'[Configuration][{Logger.__level_string__[lvl]}] {msg}')

        # default empty callbacks
        self.callbacks = BaseConfiguration.get_callbacks_dict()

        # Create a window for the configuration GUI
        if isMacOS: self.mwin = app.create_window("Configuration", 480, 1080) # macOS has a bug with window position
        else: self.mwin = app.create_window("Configuration", 480, 1080, x=0, y=30)
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
        self.reload_config_button = gui.Button("Reload")
        self.save_config_button = gui.Button("Save")
        self.apply_config_button = gui.Button("Apply")
        # -- set callbacks
        self.new_config_button.set_on_clicked(self.__new_config__)
        self.open_config_button.set_on_clicked(self.__open_config__)
        self.reload_config_button.set_on_clicked(self.__reload_config__)
        self.save_config_button.set_on_clicked(self.__save_config__)
        self.apply_config_button.set_on_clicked(self.__apply_config__)
        # -- add to button container
        button_container.add_stretch()
        button_container.add_child(self.new_config_button)
        button_container.add_child(self.open_config_button)
        button_container.add_child(self.reload_config_button)
        button_container.add_child(self.save_config_button)
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
        with open(cfg_path) as f: cfg = yaml.safe_load(f)
        return cfg
    
    def load_pipeline_algos(self):
        custom_algo_dir = os.path.join(self.last_pipeline_dir, 'algo')
        self.custom_algos_cfg = dict()
        if os.path.exists(custom_algo_dir):
            for algo_type_path in os.listdir(custom_algo_dir):
                algo_type_path = os.path.join(custom_algo_dir, algo_type_path)
                if not os.path.isdir(algo_type_path): continue
                if algo_type_path not in sys.path: sys.path.append(algo_type_path)
                algo_type = os.path.basename(algo_type_path)
                if algo_type not in self.custom_algos_cfg: self.custom_algos_cfg[algo_type] = dict()
                
                for algo_file_name in os.listdir(algo_type_path):
                    if not algo_file_name.endswith('.yml'): continue
                    self.custom_algos_cfg[algo_type].update(self.load_config(os.path.join(algo_type_path, algo_file_name)))
                self.cfg['proc'][algo_type].update(self.custom_algos_cfg[algo_type])
    
    def save_config(self, cfg, cfg_path):
        """
        Save the configuration to a YAML file.

        Args:
            cfg (dict): The configuration dictionary to be saved.
            cfg_path (str): The path to the YAML file.

        Returns:
            None
        """
        # remove custom algo configs from the main config
        if hasattr(self, 'custom_algos_cfg'):
            for algo_type in self.custom_algos_cfg:
                for algo_name in self.custom_algos_cfg[algo_type]:
                    if algo_name in self.cfg['proc'][algo_type]: self.cfg['proc'][algo_type].pop(algo_name)
                    algo_dir = os.path.join(self.last_pipeline_dir, 'algo', algo_type)
                    with open(os.path.join(algo_dir, f'{algo_name}.yml'), 'w') as f:
                        algo_config = {algo_name: self.custom_algos_cfg[algo_type][algo_name]}
                        yaml.safe_dump(algo_config, f, sort_keys=False)
        with open(cfg_path, 'w') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)

    def open_file_with_default_program(self, file_path):
        import subprocess, platform
        system_platform = platform.system()
        if system_platform == "Windows": os.startfile(file_path) # windows
        elif system_platform == "Darwin": subprocess.run(['open', file_path]) # macOS
        else: subprocess.run(['xdg-open', file_path]) # linux

    def __update_gui_from_cfg__(self, item, container, parent_keys=[], margin=0.2):
        """
        This function recursively generates the GUI from the configuration dictionary based on the item types.
        """
        G = self.generated_config_gui_dict
        data_handler_types = ['calib', 'label']
        algo_types = ['pre', 'lidar', 'camera', 'calib', 'label', 'post']
        if type(item) == dict:
            for key in item.keys():
                label_text = key
                if type(item[key]) == dict:
                    collapsable_container = gui.CollapsableVert(label_text, self.em * 0.2, gui.Margins(self.em * margin, self.em * 0.2, self.em * 0.2, self.em * 0.2))
                    if label_text in data_handler_types and parent_keys[-1] == 'data':
                        def add_data_handler_callback(algo_type=label_text):
                            import shutil
                            data_handler_dir = os.path.join(self.last_pipeline_dir, 'data_handler', algo_type)
                            os.makedirs(data_handler_dir, exist_ok=True)
                            if data_handler_dir not in sys.path: sys.path.append(data_handler_dir)
                            
                            def __input_dialog_callback__():
                                data_handler_name = getattr(self.input_dialog_widget, self.input_dialog_widget_value_variable)
                                self.__close_dialog__()
                                shutil.copy(os.path.join(resolve_for_application_root(''), 'resources', f'{algo_type}_handler_template.py'), os.path.join(data_handler_dir, f'handler_{data_handler_name}.py'))
                                self.app.run_in_thread(lambda: self.open_file_with_default_program(os.path.join(data_handler_dir, f'handler_{data_handler_name}.py')))
                            
                            self.show_input_dialog(title='Create Custom Data Handler',
                                                query=f'name(no underscores):',
                                                key=f'new_{algo_type}_data_handler',
                                                ans_base_type=gui.TextEdit,
                                                ans_type=None,
                                                value_variable='text_value',
                                                default_value=f'custom{algo_type}',
                                                custom_callback=__input_dialog_callback__)

                        add_button = gui.Button(f"Create Custom {label_text.capitalize()} Data Handler")
                        add_button.set_on_clicked(add_data_handler_callback)
                        collapsable_container.add_child(add_button)
                    elif label_text in algo_types and parent_keys[-1] == 'proc':
                        def add_data_handler_callback(algo_type=label_text):
                            import shutil
                            algo_dir = os.path.join(self.last_pipeline_dir, 'algo', algo_type)
                            os.makedirs(algo_dir, exist_ok=True)
                            def __input_dialog_callback__():
                                func_name = getattr(self.input_dialog_widget, self.input_dialog_widget_value_variable)
                                self.__close_dialog__()
                                
                                shutil.copy(os.path.join(resolve_for_application_root(''), 'resources', 'algo_func_template.py'), os.path.join(algo_dir, f'{func_name}.py'))
                                algo_txt = open(os.path.join(algo_dir, f'{func_name}.py')).read()
                                algo_txt = algo_txt.replace('AGLO_TYPE', f'AlgoType.{algo_type.lower()}')
                                algo_txt = algo_txt.replace('FUNCTION_NAME', func_name)
                                with open(os.path.join(algo_dir, f'{func_name}.py'), 'w') as f: f.write(algo_txt)
                                
                                shutil.copy(os.path.join(resolve_for_application_root(''), 'resources', 'algo_func_template.yml'), os.path.join(algo_dir, f'{func_name}.yml'))
                                conf_txt = open(os.path.join(algo_dir, f'{func_name}.yml')).read()
                                conf_txt = conf_txt.replace('FUNCTION_NAME', func_name)
                                with open(os.path.join(algo_dir, f'{func_name}.yml'), 'w') as f: f.write(conf_txt)
                                
                                self.app.run_in_thread(lambda: self.open_file_with_default_program(os.path.join(algo_dir, f'{func_name}.py')))
                                self.app.run_in_thread(lambda: self.open_file_with_default_program(os.path.join(algo_dir, f'{func_name}.yml')))
                            self.show_input_dialog(title='Create Custom Function',
                                                query=f'name:',
                                                key=f'new_{algo_type}_function_name',
                                                ans_base_type=gui.TextEdit,
                                                ans_type=None,
                                                value_variable='text_value',
                                                default_value=f'my_custom_{algo_type}_function',
                                                custom_callback=__input_dialog_callback__)

                        add_button = gui.Button(f"Create Custom {label_text.capitalize()} Function")
                        add_button.set_on_clicked(add_data_handler_callback)
                        collapsable_container.add_child(add_button)
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
                        browse_button.horizontal_padding_em = 0.5
                        browse_button.vertical_padding_em = 0
                        def create_browse_dialog(title=label_text, path_edit=G[global_key]['view']):
                            def on_done(path):
                                path_edit.text_value = path
                                self.__close_dialog__()
                            dialog_title = f"Select {title.upper()}"
                            mode = gui.FileDialog.OPEN_DIR if title.endswith('_dir') else gui.FileDialog.OPEN
                            starting_path = path_edit.text_value
                            if not os.path.isabs(starting_path): starting_path = os.path.join(self.last_pipeline_dir, starting_path)
                            if not os.path.exists(starting_path): starting_path = self.last_pipeline_dir
                            self.__show_path_dialog__(
                                title=dialog_title,
                                mode=mode,
                                initial_path=starting_path,
                                filter_type="",
                                on_done=on_done
                            )
                        browse_button.set_on_clicked(create_browse_dialog)
                        sub_container.add_fixed(0.25 * self.em)
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
                
    def __update_cfg_from_gui__(self, item, parent_keys=[], debug_key: dict=None):
        G = self.generated_config_gui_dict
        # Check if the item is a dictionary
        if type(item) == dict:
            # Iterate over all keys in the dictionary
            for key in item.keys():
                debug_key['key'] = '->'.join(parent_keys + [key])
                # Create a global key by joining the parent keys and the current key
                global_key = ".".join(parent_keys + [key])
                # If the value of the current key is a dictionary, recursively call the function
                if type(item[key]) == dict:
                    self.__update_cfg_from_gui__(item[key], parent_keys + [key], debug_key)
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
        self.app.post_to_main_thread(self.mwin, lambda: self.mwin.show_dialog(dialog))
        
    def __close_dialog__(self):
        self.showing_dialog = False
        self.mwin.close_dialog()
                
    def __show_issue_dialog__(self, issue_text):
        # create a dialog
        dialog = gui.Dialog("Configuration GUI")
        vert = gui.Vert(0, gui.Margins(self.em * 2, self.em * 2, self.em * 2, self.em * 2))
        
        msg_label = gui.Label(f'Error: {issue_text}')
        vert.add_child(msg_label)
        
        ok_button = gui.Button("OK")
        vert.add_child(ok_button)
        ok_button.set_on_clicked(lambda: self.__close_dialog__())
        
        # show the dialog
        dialog.add_child(vert)
        self.__show_dialog__(dialog)

    def show_input_dialog(self, title, query, key, ans_base_type=gui.NumberEdit, ans_type=gui.NumberEdit.INT, value_variable='int_value', default_value=0, editbox_width=200, custom_callback=None):
        # Create a dialog and base layout
        dialog = gui.Dialog(title)
        layout = gui.Horiz(margins=gui.Margins(self.em * 0.4, self.em * 0.4, self.em * 0.4, self.em * 0.4))
        
        label = gui.Label(query)
        layout.add_child(label)
        self.input_dialog_key = key
        if ans_type: self.input_dialog_widget = ans_base_type(ans_type)
        else: self.input_dialog_widget = ans_base_type()
        setattr(self.input_dialog_widget, value_variable, default_value)
        self.input_dialog_widget_value_variable = value_variable
        layout.add_child(self.input_dialog_widget)
        layout.add_fixed(0.25 * self.em)
        
        # Second row containing OK button
        ok_button = gui.Button("OK")
        ok_button.horizontal_padding_em = 0.5
        ok_button.vertical_padding_em = 0
        def ok_button_callback():
            self.__close_dialog__()
            try: self.cfg[self.input_dialog_key] = getattr(self.input_dialog_widget, self.input_dialog_widget_value_variable)
            except Exception as e:
                self.log(f'Failed to set configuration value: {e}', Logger.ERROR)
                self.__show_issue_dialog__(f'Failed to set {self.input_dialog_key} value. See logs for details.')
            
        if custom_callback: ok_button.set_on_clicked(custom_callback)
        else: ok_button.set_on_clicked(ok_button_callback)
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
        def create_base_pipeline(pipeline_dir):
            self.__close_dialog__()
            try:
                self.cfg = self.load_config(os.path.join(resolve_for_application_root(''), 'resources', 'config_template.yml'))
                self.cfg['data']['pipeline_dir'] = pipeline_dir
            except Exception as e:
                self.log(f'Failed to load configuration template: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to create new configuration. See logs for details.')
                return
            try:
                self.save_config(self.cfg, os.path.join(pipeline_dir, 'base_config.yml'))
                self.last_pipeline_dir = pipeline_dir
            except Exception as e:
                self.log(f'Failed to save configuration: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to create new configuration. See logs for details.')
                return
            
            try:
                # Generate the configuration GUI from the configuration dictionary
                cfg_gui = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
                self.__update_gui_from_cfg__(self.cfg, cfg_gui, ['cfg'])
                self.config_file_path_textedit.text_value = pipeline_dir
                self.generated_config.set_widget(cfg_gui)
            except Exception as e:
                self.log(f'Failed to update GUI from configuration: {e}', Logger.ERROR)
                self.__show_issue_dialog__('New configuration is created but it could not be displayed. See logs for details.')
            try:
                # Call the callback functions for the 'new_config' action
                for callback in self.callbacks['new_config']: callback(self.cfg)
            except Exception as e:
                self.log(f'Failed to call callback functions: {e}', Logger.ERROR)
                self.__show_issue_dialog__('New configuration is created but some erros occured post creation. See logs for details.')
        self.__show_path_dialog__("New Pipeline", gui.FileDialog.OPEN_DIR, resolve_for_default_workspace(''), "", "", create_base_pipeline)

    def __open_config__(self):
        # Define the function to load the configuration file, update GUI, and close the dialog
        def load_pipeline(file_path):
            pipeline_dir = os.path.dirname(file_path)
            self.config_file_path_textedit.text_value = pipeline_dir
            self.last_pipeline_dir = pipeline_dir
            self.__close_dialog__()
            try:
                self.cfg = self.load_config(file_path)
                self.cfg['data']['pipeline_dir'] = pipeline_dir
            except Exception as e:
                self.log(f'Failed to load configuration: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to load configuration. See logs for details.')
                return
            try:
                self.load_pipeline_algos()
                for algo_type in self.cfg['proc']:
                    algo_priorities = zip(
                        [algo_name for algo_name in self.cfg['proc'][algo_type]],
                        [algo_cfg['priority'] for algo_cfg in self.cfg['proc'][algo_type].values()]
                    )
                    algo_priorities = sorted(algo_priorities, key=lambda x: x[1])
                    self.cfg['proc'][algo_type] = {algo_name: self.cfg['proc'][algo_type][algo_name] for algo_name, _ in algo_priorities}
            except Exception as e:
                self.log(f'Failed to load custom added algorithms: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to load custom added algorithms. See logs for details.')
                
            try:
                cfg_gui = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
                self.__update_gui_from_cfg__(self.cfg, cfg_gui, ['cfg'])
                self.generated_config.set_widget(cfg_gui)
            except Exception as e:
                self.log(f'Failed to update GUI from configuration: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to update GUI from configuration. See logs for details.')
            try:
                # Call the callback functions for the 'open_config' action
                for callback in self.callbacks['open_config']: callback(self.cfg)
            except:
                self.log(f'Failed to call callback functions: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Configuration is loaded but some erros occured post loading. See logs for details.')
        
        # make sure the last workspace directory exists
        if not hasattr(self, 'last_pipeline_dir'):self.last_pipeline_dir = resolve_for_default_workspace('')
        # Open the configuration file by asking the user for the path
        self.__show_path_dialog__("Open Configuration", gui.FileDialog.OPEN, self.last_pipeline_dir, ".yml", "LiGuard base_config.yml File (*.yml)", load_pipeline)

    def __reload_config__(self):
        if not hasattr(self, 'cfg'):
            self.__show_issue_dialog__('No configuration to reload. Create/Open a configuration first.')
            return
        try: self.cfg = self.load_config(os.path.join(self.cfg['data']['pipeline_dir'], 'base_config.yml'))
        except Exception as e:
            self.log(f'Failed to reload configuration: {e}', Logger.ERROR)
            self.__show_issue_dialog__('Failed to reload configuration. See logs for details.')
            return
        try:
            self.load_pipeline_algos()
            for algo_type in self.cfg['proc']:
                algo_priorities = zip(
                    [algo_name for algo_name in self.cfg['proc'][algo_type]],
                    [algo_cfg['priority'] for algo_cfg in self.cfg['proc'][algo_type].values()]
                )
                algo_priorities = sorted(algo_priorities, key=lambda x: x[1])
                self.cfg['proc'][algo_type] = {algo_name: self.cfg['proc'][algo_type][algo_name] for algo_name, _ in algo_priorities}
        except Exception as e:
            self.log(f'Failed to load custom added algorithms: {e}', Logger.ERROR)
            self.__show_issue_dialog__('Failed to load custom added algorithms. See logs for details.')
        try:        
            cfg_gui = gui.Vert(self.em * 0.2, gui.Margins(self.em * 0.2, self.em * 0.2, self.em * 0.2, self.em * 0.2))
            self.__update_gui_from_cfg__(self.cfg, cfg_gui, ['cfg'])
            self.generated_config.set_widget(cfg_gui)
        except Exception as e:
            self.log(f'Failed to update GUI from configuration: {e}', Logger.ERROR)
            self.__show_issue_dialog__('Failed to update GUI from configuration. See logs for details.')
        try:
            # Call the callback functions for the 'open_config' action
            for callback in self.callbacks['open_config']: callback(self.cfg)
        except:
            self.log(f'Failed to call callback functions: {e}', Logger.ERROR)
            self.__show_issue_dialog__('Configuration is reloaded but some erros occured post reloading. See logs for details.')
    
    def __save_config__(self):
        if hasattr(self, 'cfg'):
            # Update the configuration from the GUI and save it to the specified path
            debug_key = dict()
            try: self.__update_cfg_from_gui__(self.cfg, ['cfg'], debug_key)
            except Exception as e:
                self.log(f'Failed to parse configuration at {debug_key["key"] if "key" in debug_key else "unknown location"}: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to save configuration. See logs for details.')
                return
            try: self.save_config(self.cfg, os.path.join(self.last_pipeline_dir, 'base_config.yml'))
            except:
                self.log(f'Failed to save configuration: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to save configuration. See logs for details.')
            try:
                # Call the callback functions for the 'save_config' action
                for callback in self.callbacks['save_config']: callback(self.cfg)
            except:
                self.log(f'Failed to call callback: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Configuration is saved but some erros occured post saving. See logs for details')
        else:
            self.__show_issue_dialog__('No configuration to save. Create/Open a configuration first.')
            
    def __apply_config__(self):
        # If the configuration exists
        if hasattr(self, 'cfg'):
            # Update the configuration from the GUI
            debug_key = dict()
            try: self.__update_cfg_from_gui__(self.cfg, ['cfg'], debug_key)
            except Exception as e:
                self.log(f'Failed to parse configuration at {debug_key["key"] if "key" in debug_key else "unknown location"}: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Failed to parse configuration. See logs for details.')
                return
            try:
                # Call the callback functions for the 'apply_config' action
                def run_call_backs():
                    for callback in self.callbacks['apply_config']: callback(self.cfg)
                threading.Thread(target=run_call_backs).start()
            except Exception as e:
                self.log(f'Failed to call callback: {e}', Logger.ERROR)
                self.__show_issue_dialog__('Configuration might not be applied completely. See logs for details.')
        else:
            self.__show_issue_dialog__('No configuration to apply. Create/Open a configuration first.')
        
    def __quit_config__(self):
        for callback in self.callbacks['quit_config']: callback()
        self.app.quit()
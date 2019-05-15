import os

import toml

from panda3d.core import Filename


class ModelData:
    def get_value(self, name, spec_node, specs, default=None):
        if name in specs:
            return specs[name]
        else:
            spec_str = spec_node.get_tag(name)
            if spec_str != '':
                spec_val = float(spec_str)
            else:
                if default is not None:
                    spec_val = default
                else:
                    raise ValueError("No value '{}' in file or model {}, and "
                                     "no default.".format(
                                         name,
                                         self.model_name,
                                     )
                    )
            specs[name] = spec_val
            return spec_val

    def __init__(self, model, model_name, asset_type):
        self.model_name = model_name
        fn_p = Filename.expand_from(
            '$MAIN_DIR/assets/{}/{}/{}.toml'.format(
                asset_type, model_name, model_name,
            )
        )
        fn = fn_p.to_os_specific()
        self.specs = {}
        file_exists = os.path.isfile(fn)
        if file_exists:
            with open(fn, 'r') as f:
                self.specs = toml.load(f)

        self.read_model(model, model_name, self.specs)

        if not file_exists:
            with open(fn, 'w') as f:
                toml.dump(self.specs, f)

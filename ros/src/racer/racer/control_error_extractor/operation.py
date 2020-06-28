class MissingOperationInput(Exception):
    def __init__(self, missing_parameter_key, op_name):
        self.missing_parameter_key_ = missing_parameter_key
        self.op_name_ = op_name

class Operation:
    def _check_input(self, input, keys):        
        for key in keys:
            if key not in input:
                raise MissingOperationInput(key, self.__class__.__name__)        

    def _run(self, input):
        pass

    def run(self, input, visualizer=None):
        self._run(input)
        if (visualizer):
            visualizer.visualize(self.__class__.__name__, input)  
class parent:
    def __init__(self):
        print("create the child class")
        self.a = 2
        self.set_a_callback_func(self.example_callback)

    def do_the_callback(self, example_goal):
        self.callback(example_goal)

    def set_a_callback_func(self, callback_func):
        self.callback = callback_func
        print("callback function set")

    def example_callback(self, example_goal):
        print("child callback sets some variables")
        self.b = example_goal
        self.c = example_goal * 2

    

class child(parent):
    def __init__(self):
        super().__init__()
        print("create the parent class")
        self.a = 1


    def example_callback(self, example_goal):
        super().example_callback(example_goal)
        print("parent callback does real work with the goal")
        self.a += example_goal


c = child()
print(c.a)
c.do_the_callback(5)
print(c.a, c.b, c.c)

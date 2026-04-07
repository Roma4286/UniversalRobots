from pynput.mouse import Button, Controller

def mouse_pressing():
    mouse = Controller()
    mouse.press(Button.left)
def mouse_releasing():
    mouse = Controller()
    mouse.release(Button.left)
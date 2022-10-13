import threading
from inputs import get_gamepad


d = {
    "L_X": 0,
    "L_Y": 0,
    "R_X": 0,
    "R_Y": 0,
    "L_T": 0,
    "R_T": 0,
    "A": 0,
    "B": 0,
    "X": 0,
    "Y": 0
}

ABS_STATE_MAX_STICK = 2**15/2**6
ABS_STATE_MAX_TRIG = 1

def gamepad_p():
    while True:
        events = get_gamepad()
        for event in events:
            if event.ev_type == "Absolute":
                if event.code == "ABS_X":
                    d["L_X"] = event.state/ABS_STATE_MAX_STICK - 1
                elif event.code == "ABS_Y":
                    d["L_Y"] = event.state/ABS_STATE_MAX_STICK + 2
                elif event.code == "ABS_RX":
                    d["R_X"] = event.state/ABS_STATE_MAX_STICK + 3
                elif event.code == "ABS_RY":
                    d["R_Y"] = event.state/ABS_STATE_MAX_STICK + 1
                elif event.code == "ABS_Z":
                    d["L_T"] = event.state/ABS_STATE_MAX_TRIG
                elif event.code == "ABS_RZ":
                    d["R_T"] = event.state/ABS_STATE_MAX_TRIG
                
            elif event.ev_type == "Key":
                if event.code == "BTN_SOUTH":
                    d["A"] = event.state
                if event.code == "BTN_EAST":
                    d["B"] = event.state
                if event.code == "BTN_WEST":
                    d["X"] = event.state
                if event.code == "BTN_NORTH":
                    d["Y"] = event.state


def start():
    gamepad_process = threading.Thread(target=gamepad_p)
    gamepad_process.daemon = True
    gamepad_process.start()


if __name__ == "__main__":
    start()

    while True:
        input()
        print(d)

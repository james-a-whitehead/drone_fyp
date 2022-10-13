import datetime
import socket
from time import sleep
import multiprocessing

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import controller


UDP_IP = "192.168.137.137"
UDP_PORT = 3333

from itertools import count

def animate(i, q, x_data, y_data, t_data, line0, line1, arr_len, jfst):
    while not q.empty():
        val = q.get()
        t_data.append(datetime.datetime.timestamp(datetime.datetime.now())-jfst)
        x_data.append(val[0])
        y_data.append(val[1])
    
    x_data = x_data[-arr_len:]
    y_data = y_data[-arr_len:]
    t_data = list(range(len(x_data)))
    
    line0.set_data(t_data, x_data)
    line1.set_data(t_data, y_data)
    
    return line0, line1


def start_animation(q):
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    x_data, y_data, t_data = [], [], []
    arr_len = 3000

    jfst = datetime.datetime.timestamp(datetime.datetime.now())

    ax.set_ylim([-90, 90])
    ax.set_xlim([0, 3000])

    line0, = ax.plot(t_data, x_data, color="r")
    line1, = ax.plot(t_data, y_data, color="g")
    line0.set(gapcolor=None)

    ani = FuncAnimation(fig, animate, fargs=(q, x_data, y_data, t_data, line0, line1, arr_len, jfst), interval=100, blit=True)
    plt.show()


def recv_messages(sock, q):
    while True:
        sleep(0.005)
        msg = sock.recvfrom(4096)[0].decode('utf-8')
        
        if msg.startswith("Delay Val: "):
            msg_trim = msg[len("Delay Val: "):]
            q.put((float(msg_trim), None))
            print(msg)

        elif msg.startswith("Orientation: "):
            msg_trim = msg[len("Orientation: "):]
            vals = msg_trim.split(",")
            q.put((float(vals[0]), float(vals[1])))
        
        else:
            print(msg)
        

        
        




def handle_input(sock, t1, t2):

    k = input()

    if k == 'k':
        try:
            a = input("kp (0), ki (1) or setpoint (2): ")
            if (a == '2') :
                b = input("rt (0), pt (1): ")
            elif (a in ['0', '1']):
                b = input("rate (0), angle (1), or yaw(2): ")
            else:
                return
            c = input("value: ")
            msg = cmd_dict[k] + " " + a + " " + b + " " + c
            #print("Sent command:", msg)
            sock.sendto(bytes(msg, "utf-8"), (UDP_IP, UDP_PORT))
        except EOFError:
            return
    
    elif k in cmd_dict:
        print("Sent command:", cmd_dict[k], k)
        sock.sendto(bytes(cmd_dict[k], "utf-8"), (UDP_IP, UDP_PORT))
    
  
    elif k == '1':
        t1.kill()
        if t2.is_alive():
            t2.kill()
        sock.shutdown(socket.SHUT_RDWR)
        sock.close()
        exit()
    
    elif k == '0':
        # controller mode
        # take control of execution
        d = controller.d
        end = 0
        while not end:

            ctrl_state = [d["L_X"], d["L_Y"], d["R_X"], d["R_Y"], d["R_T"]]
            end = d["B"]
            stop = d["A"]

            if not (stop and (last_a_state == 1)) and stop:
                # not two stop signals in a row, and stop
                msg = '8' # shutdown motors
            else:
                tmp = " ".join(["{:+5d}".format(int(round(x, 0))) for x in ctrl_state])
                msg = " ".join(("12", tmp))
            

            #print("Sent command:", msg)
            sleep(0.02)
            sock.sendto(bytes(msg, "utf-8"), (UDP_IP, UDP_PORT))
            last_a_state = stop





if __name__ == "__main__":

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_dict = {
        "s": "3", # reboot
        "u": "0", # thrust up
        "d": "1", # thrust down
        "b": "2", # battery
        "m": "4", # imu
        "g": "5", # begin balance
        "k": "6", # tune mode
        "p": "7", # print tune values
        " ": "8", # emergency motor override
        "o": "9", # toggle ota next reboot
        "t": "10", # toggle coarse/fine thrust
        "h": "11", # toggle height display
        "i": "12" # control state
    }

    q = multiprocessing.SimpleQueue()
    
    t1 = multiprocessing.Process(target=recv_messages, args=(sock, q))
    t1.start()

    t2 = multiprocessing.Process(target=start_animation, args=(q,))
    t2.start()

    controller.start()

    sock.bind(('', 3333))

    while (True):
        handle_input(sock, t1, t2)
        sleep(0.2)
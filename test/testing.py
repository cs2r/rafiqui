import time
import tkinter as tk
from rafiqui.robot import Robot


def create(frame, side):
    # Functions for scales

    def adjust_neck_x(value):
        print(f"neck_x adjusted to {value}")
        myRobot.neck_x.set_position(int(scale_neck_x.get()))

    def adjust_neck_y(value):
        print(f"neck_y adjusted to {value}")
        myRobot.neck_y.set_position(int(scale_neck_y.get()))

    def adjust_neck_z(value):
        print(f"neck_z adjusted to {value}")
        myRobot.neck_z.set_position(int(scale_neck_z.get()))

    def adjust_y_shoulder(value):
        print(f"y_shoulder adjusted to {value}")
        if side == "right":
            myRobot.r_y_shoulder.set_position(int(scale_y_shoulder.get()))
        else:
            myRobot.l_y_shoulder.set_position(int(scale_y_shoulder.get()))

    def adjust_x_shoulder(value):
        print(f"x_shoulder adjusted to {value}")
        if side == "right":
            myRobot.r_x_shoulder.set_position(int(scale_x_shoulder.get()))
        else:
            myRobot.l_x_shoulder.set_position(int(scale_x_shoulder.get()))

    def adjust_z_shoulder(value):
        print(f"z_shoulder adjusted to {value}")
        if side == "right":
            myRobot.r_z_shoulder.set_position(int(scale_z_shoulder.get()))
        else:
            myRobot.l_z_shoulder.set_position(int(scale_z_shoulder.get()))

    def adjust_elbow(value):
        # print(f"elbow adjusted to {value}")
        if side == "right":
            myRobot.r_elbow_forearm.set_position(int(scale_elbow.get()), int(scale_forearm.get()))
        else:
            myRobot.l_elbow_forearm.set_position(int(scale_elbow.get()), int(scale_forearm.get()))

    def adjust_forearm(value):
        # print(f"elbow adjusted to {value}")
        if side == "right":
            myRobot.r_elbow_forearm.set_position(int(scale_elbow.get()), int(scale_forearm.get()))
        else:
            myRobot.l_elbow_forearm.set_position(int(scale_elbow.get()), int(scale_forearm.get()))

    def adjust_palm_h(value):
        print(f"palm_h adjusted to {value}")
        if side == "right":
            myRobot.r_palm_h.set_position(int(scale_palm_h.get()))
        else:
            myRobot.l_palm_h.set_position(int(scale_palm_h.get()))

    def adjust_palm_v(value):
        print(f"epalm_v adjusted to {value}")
        if side == "right":
            myRobot.r_palm_v.set_position(int(scale_palm_v.get()))
        else:
            myRobot.l_palm_v.set_position(int(scale_palm_v.get()))


    # Functions for buttons
    def release_action():
        print("releasing " + side + " Arm")
        if side == "right":
            for att_name in dir(myRobot.right_arm):
                motor = getattr(myRobot.right_arm, att_name)
                if isinstance(motor, Robot.DXMotor) or isinstance(motor, Robot.Joint):
                    motor.set_torque(False)
        else:
            for att_name in dir(myRobot.left_arm):
                motor = getattr(myRobot.left_arm, att_name)
                if isinstance(motor, Robot.DXMotor) or isinstance(motor, Robot.Joint):
                    motor.set_torque(False)

    def reset_action():
        if side == "right":
            print(f"r_y_shoulder:  {myRobot.r_y_shoulder.get_position()} ,r_x_shoulder:  {myRobot.r_x_shoulder.get_position()} ,r_z_shoulder:  {myRobot.r_z_shoulder.get_position()} ,r_elbow_1:  {myRobot.r_elbow_1.get_position()} , r_elbow_2: {myRobot.r_elbow_2.get_position()}")
        else:
            print(f"l_y_shoulder:  {myRobot.l_y_shoulder.get_position()} ,l_x_shoulder:  {myRobot.l_x_shoulder.get_position()} ,l_z_shoulder:  {myRobot.l_z_shoulder.get_position()} ,l_elbow_1:  {myRobot.l_elbow_1.get_position()} , l_elbow_2: {myRobot.l_elbow_2.get_position()}")

    def reboot_action():
        print("Rebooting the " + side + " arm")
        if side == "right":
            for att_name in dir(myRobot.right_arm):
                motor = getattr(myRobot.right_arm, att_name)
                if isinstance(motor, Robot.Motor) or isinstance(motor, Robot.Joint):
                    motor.reboot()
        else:
            for att_name in dir(myRobot.left_arm):
                motor = getattr(myRobot.left_arm, att_name)
                if isinstance(motor, Robot.Motor) or isinstance(motor, Robot.Joint):
                    motor.reboot()

    def update_load_labels():
        if side == "right":
            load_1 = myRobot.r_y_shoulder.get_load()
            load_2 = myRobot.r_x_shoulder.get_load()
            load_3 = myRobot.r_z_shoulder.get_load()
            load_4 = myRobot.r_elbow_1.get_load()
            load_5 = myRobot.r_elbow_2.get_load()
        else:
            load_1 = myRobot.l_y_shoulder.get_load()
            load_2 = myRobot.l_x_shoulder.get_load()
            load_3 = myRobot.l_z_shoulder.get_load()
            load_4 = myRobot.l_elbow_1.get_load()
            load_5 = myRobot.l_elbow_2.get_load()

        label_load_1.config(text=f"y_shoulder Load: {load_1}%")
        label_load_2.config(text=f"x_shoulder Load: {load_2}%")
        label_load_3.config(text=f"z_shoulder Load: {load_3}%")
        label_load_4.config(text=f"elbow_1 Load: {load_4}%")
        label_load_5.config(text=f"elbow_2 Load: {load_5}%")
        frame.after(500, update_load_labels)

    if side == "top":
        # Add scales to the left frame
        scale_neck_x = tk.Scale(frame, label="neck_x", from_=0, to=1000, orient="horizontal", command=adjust_neck_x, length=400)
        scale_neck_x.pack(pady=5, fill="x", expand=True)
        scale_neck_x.set(500)

        scale_neck_y = tk.Scale(frame, label="neck_y", from_=0, to=1000, orient="horizontal", command=adjust_neck_y, length=400)
        scale_neck_y.pack(pady=5, fill="x", expand=True)
        scale_neck_y.set(500)

        scale_neck_z = tk.Scale(frame, label="neck_z", from_=0, to=1000, orient="horizontal", command=adjust_neck_z, length=400)
        scale_neck_z.pack(pady=5, fill="x", expand=True)
        scale_neck_z.set(500)
    else:
        scale_y_shoulder = tk.Scale(frame, label="y_shoulder", from_=-90, to=90, orient="horizontal", command=adjust_y_shoulder, length=400)
        scale_y_shoulder.pack(pady=5, fill="x", expand=True)
        scale_y_shoulder.set(0)

        scale_x_shoulder = tk.Scale(frame, label="x_shoulder", from_=-5, to=130, orient="horizontal", command=adjust_x_shoulder, length=400)
        scale_x_shoulder.pack(pady=5, fill="x", expand=True)
        scale_x_shoulder.set(0)

        scale_z_shoulder = tk.Scale(frame, label="z_shoulder", from_=-90, to=90, orient="horizontal", command=adjust_z_shoulder, length=400)
        scale_z_shoulder.pack(pady=5, fill="x", expand=True)
        scale_z_shoulder.set(0)

        scale_elbow = tk.Scale(frame, label="elbow", from_=0, to=120, orient="horizontal", command=adjust_elbow, length=400)
        scale_elbow.pack(pady=5, fill="x", expand=True)

        scale_forearm = tk.Scale(frame, label="forearm", from_=0, to=180, orient="horizontal", command=adjust_forearm, length=400)
        scale_forearm.pack(pady=5, fill="x", expand=True)

        scale_palm_h = tk.Scale(frame, label="palm_h", from_=0, to=1000, orient="horizontal", command=adjust_palm_h, length=400)
        scale_palm_h.pack(pady=5, fill="x", expand=True)
        scale_palm_h.set(500)

        scale_palm_v = tk.Scale(frame, label="palm_v", from_=0, to=1000, orient="horizontal", command=adjust_palm_v, length=400)
        scale_palm_v.pack(pady=5, fill="x", expand=True)
        scale_palm_v.set(500)

        # Add buttons to the right frame
        button_release = tk.Button(frame, text="Release", command=release_action)
        button_release.pack(pady=10, fill="x", expand=True)

        button_reset = tk.Button(frame, text="Reset", command=reset_action)
        button_reset.pack(pady=10, fill="x", expand=True)

        button_reboot = tk.Button(frame, text="Reboot", command=reboot_action)
        button_reboot.pack(pady=10, fill="x", expand=True)

        label_load_1 = tk.Label(frame, text="y_shoulder Load: 0%", font=("Helvetica", 10))
        label_load_1.pack(pady=5, fill="x", expand=True)

        label_load_2 = tk.Label(frame, text="x_shoulder Load: 0%", font=("Helvetica", 10))
        label_load_2.pack(pady=5, fill="x", expand=True)

        label_load_3 = tk.Label(frame, text="z_shoulder Load: 0%", font=("Helvetica", 10))
        label_load_3.pack(pady=5, fill="x", expand=True)

        label_load_4 = tk.Label(frame, text="elbow_1 Load: 0%", font=("Helvetica", 10))
        label_load_4.pack(pady=5, fill="x", expand=True)

        label_load_5 = tk.Label(frame, text="elbow_2 Load: 0%", font=("Helvetica", 10))
        label_load_5.pack(pady=5, fill="x", expand=True)

        update_load_labels()


if __name__ == "__main__":
    myRobot = Robot("config.json")
    time.sleep(0.5)
    root = tk.Tk()
    root.title("Robot Control")
    top_frame = tk.Frame(root, bg="lightblue", width=200, height=300)
    top_frame.pack(side=tk.TOP, fill=tk.BOTH)

    left_frame = tk.Frame(root, width=100, height=400)
    left_frame.pack(side="left", fill="both", expand=True)

    right_frame = tk.Frame(root, width=100, height=400)
    right_frame.pack(side="right", fill="both", expand=True)

    create(top_frame, "top")
    create(right_frame, "right")
    create(left_frame, "left")

    # Start the Tkinter event loop
    root.mainloop()
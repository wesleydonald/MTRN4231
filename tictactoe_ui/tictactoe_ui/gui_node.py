#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger

# Import the Tkinter library
import tkinter as tk
from tkinter import font

# Define our colors and fonts for a clean look
class Style:
    BG_COLOR = "#f7f9fc"
    SURFACE_COLOR = "#ffffff"
    PRIMARY_COLOR = "#3b82f6"
    PRIMARY_LIGHT = "#eff6ff"
    SUCCESS_COLOR = "#10b981"
    DANGER_COLOR = "#ef4444"
    TEXT_PRIMARY = "#111827"
    TEXT_SECONDARY = "#6b7280"
    
    TITLE_FONT = ("Inter", 18, "bold")
    STATUS_FONT = ("Inter", 16, "bold")
    METRIC_FONT = ("Inter", 22, "bold")
    METRIC_LABEL_FONT = ("Inter", 10)
    BUTTON_FONT = ("Inter", 12, "bold")
    ESTOP_FONT = ("Inter", 14, "bold")

class TictactoeGUI(tk.Frame):
    """
    This class creates the Tkinter GUI window.
    It holds all the buttons and labels, but does NOT contain any ROS 2 code.
    """
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        master.title("Tic-Tac-Toe Therapy")
        master.configure(bg=Style.BG_COLOR)

        # This dictionary will hold "pointers" to the GUI elements
        # so the ROS node can update them easily.
        self.widgets = {}

        self.create_widgets()

    def create_widgets(self):
        # --- Main Container ---
        main_frame = tk.Frame(self.master, bg=Style.SURFACE_COLOR, padx=25, pady=25)
        main_frame.pack(padx=20, pady=20, fill="x", expand=True)
        # main_frame.pack_propagate(False) # <-- REMOVE THIS LINE
 
        # --- Status ---
        self.widgets['status'] = tk.Label(
            main_frame,
            text="Connecting to robot...",
            font=Style.STATUS_FONT,
            bg=Style.PRIMARY_LIGHT,
            fg=Style.PRIMARY_COLOR,
            pady=15
        )
        self.widgets['status'].pack(fill="x", pady=(0, 20))

        # --- Metrics Frame ---
        metrics_frame = tk.Frame(main_frame, bg=Style.SURFACE_COLOR)
        metrics_frame.pack(fill="x", pady=10)
        
        # --- CHANGED: Swapped order of Ties and Losses ---
        self.widgets['wins'] = self.create_metric_box(metrics_frame, "Wins", 0)
        self.widgets['ties'] = self.create_metric_box(metrics_frame, "Ties", 1)
        self.widgets['losses'] = self.create_metric_box(metrics_frame, "Losses", 2)
        
        metrics_frame.grid_columnconfigure((0, 1, 2), weight=1)

        # --- Button Frame ---
        button_frame = tk.Frame(main_frame, bg=Style.SURFACE_COLOR)
        button_frame.pack(fill="x", pady=10)

        self.widgets['btn_start'] = tk.Button(
            button_frame,
            text="Start New Game",
            font=Style.BUTTON_FONT,
            bg=Style.SUCCESS_COLOR,
            fg="white",
            relief="flat",
            padx=15,
            pady=10
        )
        self.widgets['btn_start'].pack(side="left", fill="x", expand=True, padx=5)

        self.widgets['btn_end'] = tk.Button(
            button_frame,
            text="End Game",
            font=Style.BUTTON_FONT,
            bg=Style.TEXT_SECONDARY,
            fg="white",
            relief="flat",
            padx=15,
            pady=10
        )
        self.widgets['btn_end'].pack(side="left", fill="x", expand=True, padx=5)

        # --- E-Stop Button ---
        self.widgets['btn_estop'] = tk.Button(
            main_frame,
            text="EMERGENCY STOP",
            font=Style.ESTOP_FONT,
            bg=Style.DANGER_COLOR,
            fg="white",
            relief="flat",
            pady=15
        )
        self.widgets['btn_estop'].pack(fill="x", pady=20)

    def create_metric_box(self, parent, label, col):
        # --- FIX: Changed "bd=1, relief="solid", bordercolor=..." ---
        # --- to "highlightbackground=..., highlightthickness=1" ---
        frame = tk.Frame(parent, bg=Style.BG_COLOR, highlightbackground="#e5e7eb", highlightthickness=1)
        frame.grid(row=0, column=col, padx=10, sticky="nsew")
        
        tk.Label(
            frame,
            text=label,
            font=Style.METRIC_LABEL_FONT,
            bg=Style.BG_COLOR,
            fg=Style.TEXT_SECONDARY
        ).pack(pady=(10, 2))
        
        metric_label = tk.Label(
            frame,
            text="0", # <-- CHANGED: Removed "%"
            font=Style.METRIC_FONT,
            bg=Style.BG_COLOR,
            fg=Style.TEXT_PRIMARY
        )
        metric_label.pack(pady=(0, 10))
        
        return metric_label


class BrainGuiNode(Node):
    """
    This class is the ROS 2 Node. It is a "dumb" display.
    1. It creates the TictactoeGUI object.
    2. It sends commands from buttons to ROS 2 topics.
    3. It listens to ROS 2 topics and updates the GUI labels.
    It does NO local calculations.
    """
    def __init__(self, gui_app):
        super().__init__('brain_gui_node')
        self.gui = gui_app
        self.widgets = self.gui.widgets

        # --- REMOVED: All local game logic variables ---
        # self.games_played, self.wins, self.losses, self.ties, self.game_is_active

        # --- Publishers (GUI Node -> Robot) ---
        self.control_pub = self.create_publisher(String, '/game/control', 10)
        self.estop_service = self.create_client(Trigger, '/ur5e/trigger_estop')

        # --- Subscribers (Robot -> GUI Node) ---
        
        # Listens for status strings like "It's your turn!"
        self.status_sub = self.create_subscription(
            String,
            '/game/status',
            self.status_callback,
            10)
            
        # NEW: Listens for the total WIN count from brain.cpp
        self.wins_sub = self.create_subscription(
            Int32,
            '/game/metrics/wins',
            self.wins_callback,
            10)
            
        # NEW: Listens for the total LOSS count from brain.cpp
        self.losses_sub = self.create_subscription(
            Int32,
            '/game/metrics/losses',
            self.losses_callback,
            10)
            
        # NEW: Listens for the total TIE count from brain.cpp
        self.ties_sub = self.create_subscription(
            Int32,
            '/game/metrics/ties',
            self.ties_callback,
            10)
        
        # --- REMOVED: game_result_sub ---
        
        # --- Connect GUI Buttons to ROS Functions ---
        self.widgets['btn_start'].configure(command=self.on_start_click)
        self.widgets['btn_end'].configure(command=self.on_end_click)
        self.widgets['btn_estop'].configure(command=self.on_estop_click)
        
        self.get_logger().info("GUI Node started. Waiting for game_logic...")
        self.widgets['status'].configure(text="Waiting for robot...")


    # --- Button Click Handlers (Publish to ROS) ---

    def on_start_click(self):
        self.get_logger().info('"Start Game" clicked')
        # Simply publish the command. Do not track any state.
        self.control_pub.publish(String(data="start"))
        
        # We can update the status right away for good user feedback
        self.widgets['status'].configure(
            text="New game started!",
            fg=Style.PRIMARY_COLOR,
            bg=Style.PRIMARY_LIGHT
        )

    def on_end_click(self):
        self.get_logger().info('"End Game" clicked')
        # Simply publish the command.
        self.control_pub.publish(String(data="end"))
        
        # When user ends session, we can reset the GUI labels to 0
        # The C++ node will also receive the "end" and reset its internal counts.
        self.widgets['wins'].configure(text="0")
        self.widgets['losses'].configure(text="0")
        self.widgets['ties'].configure(text="0")
        self.widgets['status'].configure(
            text="Session Ended.",
            fg=Style.TEXT_SECONDARY,
            bg=Style.BG_COLOR
        )

    def on_estop_click(self):
        self.get_logger().error('!!! EMERGENCY STOP CLICKED !!!')
        
        # Update GUI immediately
        self.widgets['status'].configure(
            text="EMERGENCY STOPPED",
            fg="white",
            bg=Style.DANGER_COLOR
        )
        
        # Call the service
        if not self.estop_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('E-Stop service not available!')
            self.widgets['status'].configure(text="E-STOP FAILED!")
            return

        request = Trigger.Request()
        self.estop_service.call_async(request)
        self.get_logger().info('E-Stop request sent.')

    # --- Subscriber Callbacks (Update GUI) ---

    def status_callback(self, msg):
        self.get_logger().info(f'Status update: {msg.data}')
        # Update the status label
        self.widgets['status'].configure(text=msg.data)

    # REMOVED: game_result_callback
    # REMOVED: publish_rates

    # NEW: Callback for WIN count
    def wins_callback(self, msg):
        self.get_logger().info(f'Wins count update: {msg.data}')
        self.widgets['wins'].configure(text=f"{msg.data}")

    # NEW: Callback for LOSS count
    def losses_callback(self, msg):
        self.get_logger().info(f'Losses count update: {msg.data}')
        self.widgets['losses'].configure(text=f"{msg.data}")
        
    # NEW: Callback for TIE count
    def ties_callback(self, msg):
        self.get_logger().info(f'Ties count update: {msg.data}')
        self.widgets['ties'].configure(text=f"{msg.data}")


def main(args=None):
    rclpy.init(args=args)

    # --- This is the key part ---
    # 1. Create the Tkinter root window
    root = tk.Tk()
    root.geometry("500x600") # <-- ADD THIS LINE
    
    # 2. Create the GUI App object
    gui = TictactoeGUI(root)
    
    # 3. Create the ROS 2 Node and pass it the GUI object
    gui_node = BrainGuiNode(gui)

    # This is the magic!
    # We create a loop that checks for ROS 2 messages (spin_once)
    # and then updates the Tkinter GUI (root.update).
    def spin_and_update():
        rclpy.spin_once(gui_node, timeout_sec=0.01) # Check for ROS messages
        root.update()                               # Update the GUI
        root.after(10, spin_and_update)             # Schedule this to run again in 10ms

    try:
        # Start the loop
        spin_and_update()
        # This line is now "blocking" and will run until the user closes the window
        root.mainloop() 
        
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
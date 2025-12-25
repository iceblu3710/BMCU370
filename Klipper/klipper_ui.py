import tkinter as tk
from tkinter import ttk, colorchooser, messagebox
import serial
import serial.tools.list_ports
import json
import time
import threading

class BMCUKlipperUI:
    def __init__(self, root):
        self.root = root
        self.root.title("BMCU370 Klipper Serial Interface")
        # self.root.geometry("800x600") # Auto-fit

        
        self.serial_port = None
        self.is_connected = False
        self.packet_lock = threading.Lock()
        
        # --- Variables ---
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        
        # Movement Settings
        self.move_speed_var = tk.StringVar(value="Slow") # Fast, Slow
        self.move_mode_var = tk.StringVar(value="Distance") # Continuous, Distance
        self.move_dist_var = tk.DoubleVar(value=50.0)
        
        # Lane Data holder [Lane 0-3]
        self.lane_vars = [] 
        for i in range(4):
            self.lane_vars.append({
                'name': tk.StringVar(value=f"Lane {i+1}"),
                'color': tk.StringVar(value="#FFFFFF"),
                'temp': tk.StringVar(value="--/--"),
                'status': tk.StringVar(value="Idle"),
                'presence': tk.StringVar(value="Empty"),
                'rfid': tk.StringVar(value=""),
                'meters': tk.StringVar(value="0.0 m")
            })

        
        # Debug Console
        self.console_var = tk.StringVar()
        self.polling_enabled = tk.BooleanVar(value=False)
        
        self.create_widgets()
        
        # Start polling thread (Paused by default)
        self.stop_thread = False
        self.thread = threading.Thread(target=self.poll_loop, daemon=True)
        self.thread.start()
        
    def log(self, msg):
        print(f"[LOG] {msg}")
        # Could append to a text widget if we added one
        pass

    def create_widgets(self):
        # Top Bar: Connection
        top_frame = ttk.Frame(self.root, padding=5)
        top_frame.pack(fill=tk.X)
        
        ttk.Label(top_frame, text="Serial Port:").pack(side=tk.LEFT)
        self.port_combobox = ttk.Combobox(top_frame, textvariable=self.port_var, width=15)
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()
        
        ttk.Button(top_frame, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_btn = ttk.Button(top_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(top_frame, textvariable=self.status_var).pack(side=tk.RIGHT, padx=10)
        ttk.Checkbutton(top_frame, text="Poll", variable=self.polling_enabled).pack(side=tk.RIGHT, padx=5)
        
        # Main Grid: Lanes
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        for i in range(4):
            self.create_lane_card(main_frame, i)
            
        # Global Controls
        # Global Controls
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill=tk.X)
        
        ttk.Button(bottom_frame, text="STOP ALL", command=self.stop_all, width=20, padding=(10, 20)).pack(side=tk.LEFT)
        
        # Movement Options Frame
        move_frame = ttk.LabelFrame(bottom_frame, text="Movement Options", padding=5)
        move_frame.pack(side=tk.LEFT, padx=20, fill=tk.X, expand=True)
        
        # Speed
        ttk.Radiobutton(move_frame, text="Slow", variable=self.move_speed_var, value="Slow").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(move_frame, text="Fast", variable=self.move_speed_var, value="Fast").pack(side=tk.LEFT, padx=5)
        
        # Spacer
        ttk.Separator(move_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        # Mode
        ttk.Radiobutton(move_frame, text="Continuous", variable=self.move_mode_var, value="Continuous").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(move_frame, text="Distance", variable=self.move_mode_var, value="Distance").pack(side=tk.LEFT, padx=5)
        
        # Distance Spinbox
        ttk.Label(move_frame, text="Dist:").pack(side=tk.LEFT, padx=2)
        dist_spin = tk.Spinbox(move_frame, from_=0.1, to=1000.0, increment=10.0, textvariable=self.move_dist_var, width=8)
        dist_spin.pack(side=tk.LEFT, padx=5)
        ttk.Label(move_frame, text="mm").pack(side=tk.LEFT)

    def create_lane_card(self, parent, lane_idx):
        card = ttk.LabelFrame(parent, text=f"Lane {lane_idx+1}", padding=10)
        card.grid(row=0, column=lane_idx, padx=5, pady=5, sticky="nsew")
        parent.columnconfigure(lane_idx, weight=1)
        
        vars = self.lane_vars[lane_idx]
        
        # Filament Info
        info_frame = ttk.Frame(card)
        info_frame.pack(fill=tk.X, pady=5)
        
        # Color Box
        self.color_box = tk.Label(info_frame, bg="white", width=4, relief="solid")
        self.color_box.pack(side=tk.LEFT, padx=5)
        # Store widget ref to update color later logic
        vars['color_widget'] = self.color_box 
        
        ttk.Label(info_frame, textvariable=vars['name'], font=('Arial', 10, 'bold')).pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        ttk.Label(card, textvariable=vars['rfid'], font=('Arial', 8)).pack(pady=1)
        ttk.Label(card, textvariable=vars['meters']).pack(pady=1)
        ttk.Label(card, textvariable=vars['temp'], borderwidth=1).pack(pady=2)
        ttk.Label(card, textvariable=vars['presence'], foreground="blue").pack(pady=2)
        ttk.Label(card, textvariable=vars['status']).pack(pady=2)
        
        # Edit Button
        ttk.Button(card, text="Edit Filament", command=lambda i=lane_idx: self.edit_filament(i)).pack(fill=tk.X, pady=2)
        
        # Controls
        ctrl_frame = ttk.Frame(card)
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(ctrl_frame, text="FEED", command=lambda i=lane_idx: self.feed_lane(i), width=6).pack(side=tk.LEFT, padx=1)
        ttk.Button(ctrl_frame, text="RETRACT", command=lambda i=lane_idx: self.retract_lane(i), width=9).pack(side=tk.LEFT, padx=1)
        ttk.Button(ctrl_frame, text="AUTO", command=lambda i=lane_idx: self.auto_feed_lane(i), width=6).pack(side=tk.LEFT, padx=1)
        
        # Load/Unload
        action_frame = ttk.Frame(card)
        action_frame.pack(fill=tk.X, pady=2)
        # These will use MOVE for now as generic load/unload might be complex via generic "MOVE" without state handling override?
        # Ideally using LOAD_FILAMENT / UNLOAD_FILAMENT commands from firmware if exposed?
        # Firmware has StartLoadFilament but not exposed as Klipper command yet?
        # Ah, task was "run and stop feed commands". I'll use simple MOVE for now.
        
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.current(0)
            
    def toggle_connection(self):
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        port = self.port_var.get()
        if not port: return
        try:
            self.serial_port = serial.Serial(port, 250000, timeout=0.1)
            # DTR reset for nice reboot
            self.serial_port.dtr = False
            time.sleep(0.1)
            self.is_connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_var.set("Connected")
            # Initial poll - DISABLED to prevent flood
            # self.refresh_all_filaments()
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            
    def disconnect(self):
        self.is_connected = False
        if self.serial_port:
            self.serial_port.close()
        self.connect_btn.config(text="Connect")
        self.status_var.set("Disconnected")

    def send_pkt(self, cmd, args={}, wait_response=False):
        if not self.is_connected: return None
        
        pkt = {
            "id": int(time.time() * 1000) % 10000,
            "cmd": cmd,
            "args": args
        }
        
        try:
            with self.packet_lock:
                msg = json.dumps(pkt) + "\n"
                self.serial_port.write(msg.encode())
                
                if wait_response:
                    # Simple blocking read for response (POC)
                    # In production, use async reader.
                    # For now, quick hack to read line
                    resp_line = self.serial_port.readline()
                    if resp_line:
                        try:
                            # DEBUG: Print raw response to console
                            print(f"RX: {resp_line.decode().strip()}")
                            return json.loads(resp_line)
                        except Exception as e:
                            print(f"JSON Parse Error: {e} | Raw: {resp_line}")
                            return None
        except Exception as e:
            print(f"Error sending: {e}")
            self.disconnect()
        return None

    def refresh_all_filaments(self):
        for i in range(4):
            self.get_filament_info(i)

    def get_filament_info(self, lane):
        resp = self.send_pkt("GET_FILAMENT_INFO", {"lane": lane}, wait_response=True)
        if resp and resp.get("ok"):
            info = resp.get("info", {})
            vars = self.lane_vars[lane]
            
            vars['name'].set(info.get("name", "Unknown"))
            tmin = info.get("temp_min", 0)
            tmax = info.get("temp_max", 0)
            vars['temp'].set(f"{tmin}-{tmax}°C")
            
            c = info.get("color", [255, 255, 255])
            if len(c) >= 3:
                hex_col = "#{:02x}{:02x}{:02x}".format(c[0], c[1], c[2])
                vars['color'].set(hex_col)
                vars['color_widget'].config(bg=hex_col)
            
            vars['rfid'].set(info.get("id_str", ""))
            vars['meters'].set(f"{info.get('meters', 0.0):.1f} m")

    def poll_loop(self):
        while not self.stop_thread:
            if self.is_connected and self.polling_enabled.get():
                # Poll STATUS
                try:
                    resp = self.send_pkt("STATUS", {}, wait_response=True)
                    if resp and resp.get("lanes"):
                        lanes = resp['lanes']
                        for i in range(min(4, len(lanes))):
                            l = lanes[i]
                            vars = self.lane_vars[i]
                            
                            
                            vars['presence'].set("Filament!" if l.get('present') else "Empty")
                            vars['status'].set(l.get('motion', 'Idle'))
                            vars['rfid'].set(l.get('rfid', ''))
                            vars['meters'].set(f"{l.get('meters', 0.0):.1f} m")
                            
                            vars['name'].set(l.get('name', f"Lane {i+1}"))
                            
                            t_min = l.get('temp_min', 0)
                            t_max = l.get('temp_max', 0)
                            vars['temp'].set(f"{t_min}/{t_max}")
                            
                            # Color handling (Array [r,g,b,a] -> Hex)
                            c_arr = l.get('color')
                            if c_arr and isinstance(c_arr, list) and len(c_arr) >= 3:
                                r,g,b = c_arr[0], c_arr[1], c_arr[2]
                                hex_col = f"#{r:02x}{g:02x}{b:02x}"
                                vars['color'].set(hex_col)
                                # Update widget background
                                if 'color_widget' in vars:
                                    try:
                                        vars['color_widget'].config(bg=hex_col)
                                    except:
                                        pass
                                
                            # Auto Feed State Visualization
                            if l.get('motion') == "AutoFeed":
                                vars['status'].set("AUTO FEEDING")
                except Exception as e:
                    print(f"Poll Error: {e}")

                        
                # 1 Hz Poll
            time.sleep(1)

    def edit_filament(self, lane):
        if not self.is_connected: return
        
        # Simple Dialog
        win = tk.Toplevel(self.root)
        win.title(f"Edit Lane {lane+1}")
        
        vars = self.lane_vars[lane]
        
        tk.Label(win, text="Name:").grid(row=0, column=0)
        name_entry = tk.Entry(win)
        name_entry.insert(0, vars['name'].get())
        name_entry.grid(row=0, column=1)
        
        tk.Label(win, text="Temp Min:").grid(row=1, column=0, padx=5, pady=5)
        min_entry = tk.Entry(win)
        # Parse "min/max" string safely
        temp_str = vars['temp'].get()
        t_min = "220"
        t_max = "240"
        if "/" in temp_str:
            parts = temp_str.split('/')
            if len(parts) >= 2:
                t_min = parts[0]
                t_max = parts[1].replace('°C','')
        elif "-" in temp_str: # Legacy fallback
             parts = temp_str.split('-')
             if len(parts) >= 2:
                t_min = parts[0]
                t_max = parts[1].replace('°C','')
                
        min_entry.insert(0, t_min)
        min_entry.grid(row=1, column=1, padx=5, pady=5)

        tk.Label(win, text="Temp Max:").grid(row=2, column=0, padx=5, pady=5)
        max_entry = tk.Entry(win)
        max_entry.insert(0, t_max)
        max_entry.grid(row=2, column=1, padx=5, pady=5)
        
        tk.Label(win, text="RFID:").grid(row=3, column=0, padx=5, pady=5)
        rfid_entry = tk.Entry(win)
        rfid_entry.insert(0, vars['rfid'].get())
        rfid_entry.grid(row=3, column=1, padx=5, pady=5)

        tk.Label(win, text="Meters:").grid(row=4, column=0, padx=5, pady=5)
        meter_entry = tk.Entry(win)
        meter_entry.insert(0, vars['meters'].get().replace(' m',''))
        meter_entry.grid(row=4, column=1, padx=5, pady=5)
        
        col_btn = tk.Button(win, text="Pick Color", command=lambda: self.pick_color(win))
        col_btn.grid(row=5, column=0, columnspan=2)
        
        self.temp_color = None
        
        def save():
            name = name_entry.get()
            tmin = int(min_entry.get())
            tmax = int(max_entry.get())
            rfid = rfid_entry.get()
            try:
                meters = float(meter_entry.get())
            except:
                meters = 0
            
            args = {
                "lane": lane,
                "name": name,
                "temp_min": tmin,
                "temp_max": tmax,
                "id_str": rfid,
                "meters": meters
            }
            if hasattr(self, 'temp_color') and self.temp_color:
                args['color'] = self.temp_color
                
            # Wait for response so we consume the OK packet before requesting info
            self.send_pkt("SET_FILAMENT_INFO", args, wait_response=True)
            self.get_filament_info(lane) # Refresh
            win.destroy()
            
        tk.Button(win, text="Save", command=save).grid(row=6, column=0, columnspan=2)

    def pick_color(self, parent):
        color = colorchooser.askcolor(title="Choose color", parent=parent)
        if color[0]:
            # color is ((r, g, b), '#hex')
            rgb = [int(x) for x in color[0]]
            self.temp_color = rgb
            
    def feed_lane(self, lane):
        speed = 10.0 if self.move_speed_var.get() == "Slow" else 30.0
        dist = 0.0
        
        if self.move_mode_var.get() == "Continuous":
            dist = 100000.0 # Run "forever" until stop
        else:
            try:
                dist = float(self.move_dist_var.get())
            except:
                dist = 50.0
                
        self.send_pkt("MOVE", {"axis": str(lane), "dist_mm": dist, "speed": speed})
        
    def retract_lane(self, lane):
        speed = 10.0 if self.move_speed_var.get() == "Slow" else 30.0
        dist = 0.0
        
        if self.move_mode_var.get() == "Continuous":
            dist = 100000.0 
        else:
            try:
                dist = float(self.move_dist_var.get())
            except:
                dist = 50.0
                
        # Negative speed for retract? Or positive dist with negative speed?
        # Protocol says: speed
        # ControlLogic::MoveAxis uses: motors[axis].target_velocity = speed * ...
        # If speed is negative, it reverses.
        self.send_pkt("MOVE", {"axis": str(lane), "dist_mm": dist, "speed": -speed})

    def auto_feed_lane(self, lane):
        # Toggle or enable auto feed
        # We assume enabling updates state.
        self.send_pkt("SET_AUTO_FEED", {"lane": lane, "enable": True})

    def stop_all(self):
        self.send_pkt("STOP")

if __name__ == "__main__":
    root = tk.Tk()
    app = BMCUKlipperUI(root)
    root.mainloop()

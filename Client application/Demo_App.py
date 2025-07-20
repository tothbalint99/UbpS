import tkinter as tk
from tkinter import filedialog, messagebox
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from matplotlib.figure import Figure
import ctypes
import serial as sr
import time
from threading import *

class Demo(tk.Tk):
    def __init__(self):
        super().__init__()
        # Initialize variables
        self.running = False
        self.fs = 25  # Sampling frequency in Hz
        self.US_data = np.array([])
        self.num_rec = 0
        self.len_rec = 0
        self.idx_rec = 0
        self.signal = np.array([])
        self.signal_len = 0
        self.prev_response = 0
        self.valid = True
        self.sbp = 0  # Systolic Blood Pressure
        self.dbp = 0  # Diastolic Blood Pressure
        self.map = 0  # Mean Arterial Pressure
        self.t1 = Thread(target=self.data_sending_and_receiving) # Seaparate thread for data sending and aquisition
        self.signal_lock = Lock() # Lock object for protecting signal
        self.valid_lock = Lock() # Lock object for protecting valid and indexes
        self.running_lock = Lock() # Lock object for protecting running and stop_pressed
        self.on = True
        self.stop_pressed = False

        # Define communication port
        self.ser = sr.Serial('COM4', 115200, timeout=1000)

        # Create the visual design of the application
        self.title("US Blood Pressure App")
        self.config(background='white')

        # Set the window dimensions
        user32 = ctypes.windll.user32
        screen_width = user32.GetSystemMetrics(0)
        screen_height = user32.GetSystemMetrics(1)
        window_width = int(0.92 * screen_width)
        window_height = int(0.62 * screen_height)
        window_x = (screen_width - window_width) // 2
        window_y = (screen_height - window_height) // 2
        self.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")
        self.resizable(False, False)

        # Create the plot
        self.fig = Figure()
        self.ax1 = self.fig.add_subplot()
        self.ax1.set_title('Estimated Blood Pressure Waveform')
        self.ax1.set_xlabel('Time [s]')
        self.ax1.set_ylabel('Blood Pressure [mmHg]')
        self.ax1.set_xlim(0, 8)
        self.ax1.set_ylim(40, 150)
        self.lines = self.ax1.plot([], [], linestyle='-', color='g', linewidth=1, markersize=4)[0]

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().place(x=0, y=50, width=1200, height=400)
        self.canvas.draw()

        # Create start button
        self.update()
        self.start = tk.Button(self, text="Start measurement", font=('calibri', 18), command=self.button_pressed, background='#39FF14', bd=3, width=114)
        self.start.place(x=20, y=470)

        # Create title label
        self.update()
        self.l1 = tk.Label(self, text="Ultrasound-based blood pressure estimation", font=('calibri', 30), bg='white')
        self.l1.place(x=20, y=0)

        # Create author label
        self.update()
        self.name = tk.Label(self, text="Created by Bálint Tóth (Last update: 27.06.2024)", font=('calibri', 11), bg='white')
        self.name.place(x=1090, y=28)

        # Create a separator line
        self.update()
        line = tk.Canvas(self, width=window_width * 0.97, height=2, bg='black')
        line.place(x=20, y=50)

        # Create systolic pressure label
        self.update()
        self.Sys_l = tk.Label(self, text='Systolic Blood Pressure', font=('calibri', 18), bg='white')
        self.Sys_l.place(x=1110, y=100)

        # Create systolic pressure value display
        self.update()
        self.Sys_num = tk.Label(self, text="-", font=('calibri', 18), width=22, height=1, borderwidth=3, relief="solid", anchor="w", bg='white')
        self.Sys_num.place(x=self.Sys_l.winfo_x(), y=self.Sys_l.winfo_y() + self.Sys_l.winfo_reqheight() + 3)

        # Create diastolic pressure label
        self.update()
        self.Dia_l = tk.Label(self, text='Diastolic Blood Pressure', font=('calibri', 18), bg='white')
        self.Dia_l.place(x=self.Sys_l.winfo_x(), y=self.Sys_num.winfo_y() + self.Sys_num.winfo_reqheight() + 35)

        # Create diastolic pressure value display
        self.update()
        self.Dia_num = tk.Label(self, text="-", font=('calibri', 18), width=22, height=1, borderwidth=3, relief="solid", anchor="w", bg='white')
        self.Dia_num.place(x=self.Dia_l.winfo_x(), y=self.Dia_l.winfo_y() + self.Dia_l.winfo_reqheight() + 3)

        # Create message label
        self.update()
        self.Msg_l = tk.Label(self, text='Message', font=('calibri', 18), bg='white')
        self.Msg_l.place(x=self.Dia_num.winfo_x(), y=self.Dia_num.winfo_y() + self.Dia_num.winfo_reqheight() + 35)

        # Create message display
        self.update()
        self.Msg_num = tk.Label(self, text="Not started", font=('calibri', 18), width=22, height=1, borderwidth=3, relief="solid", anchor="w", bg='white')
        self.Msg_num.place(x=self.Msg_l.winfo_x(), y=self.Msg_l.winfo_y() + self.Msg_l.winfo_reqheight() + 3)

    def get_bp_values(self):
        """Create a dialog for the user to input SBP and DBP values."""
        # Creating a pop-up dialog window
        dialog = tk.Toplevel(self)
        dialog.title("Calibration")
        
        # Creating labels and entry boxes for SBP and DBP
        tk.Label(dialog, text="SBP [mmHg]:").grid(row=0, column=0, padx=10, pady=10)
        tk.Label(dialog, text="DBP [mmHg]:").grid(row=1, column=0, padx=10, pady=10)
    
        sbp_entry = tk.Entry(dialog)
        dbp_entry = tk.Entry(dialog)

        sbp_entry.grid(row=0, column=1, padx=10, pady=10)
        dbp_entry.grid(row=1, column=1, padx=10, pady=10)

        # Function to handle the submission of values
        def submit():
            sbp = sbp_entry.get()
            dbp = dbp_entry.get()

            if sbp.isdigit() and dbp.isdigit():
                self.sbp = sbp
                self.dbp = dbp
                self.map = float(dbp) + (1/3) * (float(sbp) - float(dbp))
                dialog.destroy()
            else:
                messagebox.showerror("Invalid Input", "Please enter valid numbers for SBP and DBP")
        
        # Creating submit button
        submit_button = tk.Button(dialog, text="Submit", command=submit)
        submit_button.grid(row=2, columnspan=2, pady=10)
        
        # Making the dialog modal
        dialog.transient(self)
        dialog.grab_set()
        self.wait_window(dialog)

    def load_csv(self):
        """Load the CSV file containing ultrasound data."""
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        print(file_path)
        if file_path:
            try:
                self.US_data = np.array(pd.read_csv(file_path, header=None))
                self.num_rec = len(self.US_data)
                self.len_rec = self.US_data.shape[1]
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load CSV file.\n\nError details: {str(e)}")
        
    def button_pressed(self):
        """Handle start/stop button press."""
        self.running_lock.acquire()
        if not self.running:
            self.running_lock.release()
            self.idx_rec = 0
            self.signal_len = 0

            self.signal_lock.acquire()
            self.signal = np.array([])
            self.signal_lock.release()

            self.US_data = np.array([])
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.load_csv()
            self.get_bp_values()

            # Sending start message
            message = f"s,{self.sbp},{self.dbp}"
            message = message.ljust(10)
            message_bytes = message.encode('utf-8')
            self.ser.write(message_bytes) 

            # Visualizing changes
            self.Msg_num.config(text="Starting...")
            self.Sys_num.config(text="Calculating...")
            self.Dia_num.config(text="Calculating...")
            self.start.config(text="Stop measurement", background='red')
            self.running_lock.acquire()
            self.running = True
            self.running_lock.release()
            print("Started!")
            
            if not self.t1.is_alive():
                self.t1.start()
        else:
            self.running_lock.release()
            self.start.config(text="Start measurement", background='#39FF14')
            self.running_lock.acquire()
            self.stop_pressed = True
            self.running_lock.release()

    def data_sending_and_receiving(self):
        while(self.on):
            self.running_lock.acquire()
            if(self.running):
                self.running_lock.release()
                try:
                    # Waiting for measurement aquiring meassage
                    response = self.ser.readline().strip()
                    decoded_response = response.decode('utf-8')
                    if decoded_response[0] != 'm':
                        break
                    self.ser.flush()
                    
                    self.running_lock.acquire()
                    if self.stop_pressed == True or self.num_rec-1==self.idx_rec:
                        self.running_lock.release() 
                        # Sending stop message
                        message = "e"
                        message = message.ljust(256)
                        message_bytes = message.encode('utf-8')
                        self.ser.write(message_bytes)
                        if self.stop_pressed == True:
                            print("Stopped!")
                        elif self.num_rec-1==self.idx_rec:
                            self.start.config(text="Start measurement", background='#39FF14')
                            print("Ended!")

                        self.running_lock.acquire()
                        self.stop_pressed = False
                        self.running = False
                        self.running_lock.release()
                    else:
                        self.running_lock.release()
                        # Sending data
                        message = ','.join(map(str, self.US_data[self.idx_rec][:self.len_rec]))
                        message = message.ljust(256)
                        message_bytes = message.encode('utf-8')
                        self.ser.write(message_bytes)
                        
                        # Receiving response
                        response = self.ser.readline().strip()
                        response = float(response)

                        # Counting recording number and signal number
                        self.valid_lock.acquire()
                        self.idx_rec += 1
                        self.signal_len += 1
                        self.valid_lock.release()
                        print(response)

                        if response == 0:
                            response = self.prev_response
                            self.valid_lock.acquire()
                            self.valid = False
                            self.valid_lock.release()
                        else:
                            self.valid_lock.acquire()
                            self.valid = True
                            self.valid_lock.release()

                        # Collecting the responses
                        self.valid_lock.acquire()
                        if self.signal_len > 200:
                            self.valid_lock.release()
                            self.signal_lock.acquire()
                            self.signal[:-1] = self.signal[1:]
                            self.signal[-1] = response
                            self.signal_lock.release()
                            self.prev_response = response
                        else:
                            if self.signal_len >= 2:
                                self.valid_lock.release()
                                self.signal_lock.acquire()
                                self.signal = np.append(self.signal, response)
                                self.signal_lock.release()
                                self.prev_response = response
                            else:
                                self.valid_lock.release()
                                self.signal_lock.acquire()
                                self.signal = np.append(self.signal, self.map)
                                self.signal_lock.release()
                                self.prev_response = self.map
                except Exception as e:
                    print(e)
            else:
                self.running_lock.release()
                time.sleep(0.005)

    def main_loop(self):
        """Main loop to read data from the serial port and update the plot."""
        self.running_lock.acquire()
        if self.running:
            self.running_lock.release()
            # Handling the case if the recording is over
            self.valid_lock.acquire()
            if self.idx_rec >= self.num_rec:
                self.valid_lock.release()
                self.start.config(text="Start measurement", background='#39FF14')
                self.Msg_num.config(text="Recording ended", background='light blue')
                self.running_lock.acquire()
                self.running = False
                self.stop_pressed = False
                self.running_lock.release()
            else:
                self.valid_lock.release()

                # Providing visual feedback
                self.valid_lock.acquire()
                if not self.valid:
                    self.valid_lock.release()
                    self.Msg_num.config(text="Wall-finding & Calibrating")
                    self.Msg_num.config(bg="light blue")
                    self.lines.set_color('lightgray')
                else:
                    self.valid_lock.release()
                    self.Msg_num.config(text="Correct tracking")
                    self.Msg_num.config(bg="#39FF14")
                    self.lines.set_color('g')

                # Printing and plotting results
                self.valid_lock.acquire()
                if self.signal_len > 44 and self.valid:
                    self.valid_lock.release()
                    self.signal_lock.acquire()
                    self.Sys_num.config(text='{:.1f}'.format(np.max(self.signal)) + " mmHg")
                    self.Dia_num.config(text='{:.1f}'.format(np.min(self.signal)) + " mmHg")
                    self.ax1.set_ylim(ymin=np.min(self.signal)-10, ymax=np.max(self.signal)+10)
                    self.signal_lock.release()
                else:
                    self.valid_lock.release()
                    self.Sys_num.config(text="Calculating...")
                    self.Dia_num.config(text="Calculating...")
                self.signal_lock.acquire()
                self.lines.set_xdata(np.arange(0, len(self.signal)) / self.fs)
                self.lines.set_ydata(self.signal)
                self.signal_lock.release()
            
                # Drawing the changes
                self.canvas.draw()
        else:
            self.running_lock.release()

        self.after(1, self.main_loop)


if __name__ == "__main__":
    demo = Demo()
    demo.after(1, demo.main_loop)
    demo.mainloop()
    demo.on = False
    if demo.t1.is_alive():
        demo.t1.join()
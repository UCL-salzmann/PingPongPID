
import tkinter as tk
import serial
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import numpy as np
from tkinter import ttk
import csv
from tkinter import filedialog
import os
import serial.tools.list_ports
import functools
import tkinter.messagebox

ports = serial.tools.list_ports.comports()
ser = serial.Serial()

root = tk.Tk()
root.title('PingPongPID Python GUI')
system_running = False


## CANVAS SETTINGS 

notebook = ttk.Notebook(root)
notebook.grid(row=0, column=0, columnspan=10,padx=10, pady=10)

COM_frame = ttk.Frame(notebook)
notebook.add(COM_frame, text="Connect COM Port")

main_frame = ttk.Frame(notebook)
notebook.add(main_frame, text = "Parameters")

plot_frame = ttk.Frame(notebook)
notebook.add(plot_frame, text="Data Plots")

serial_frame = ttk.Frame(notebook)
notebook.add(serial_frame, text="Serial Data")


serial_text = tk.Text(serial_frame, wrap='word')
serial_text.grid(row=0,column=0, rowspan=20, columnspan=10, padx=10, pady=10)

scrollbar = tk.Scrollbar(serial_frame, command=serial_text.yview)
scrollbar.grid(row=0, column=10, rowspan=20, sticky='ns')
serial_text['yscrollcommand'] = scrollbar.set

## COM PORT SELECTION

def initComPort(index):
	global current_port
	try:
		currentPort = str(ports[index])
		comPortVar = str(currentPort.split(' ')[0])
		if ser.is_open:
			if ser.port != comPortVar:
				print(f"Closing {ser.port} and opening {comPortVar}")
			ser.close()
		ser.port = comPortVar
		ser.baudrate = 115200
		ser.open()
		print(f"Opened COM port: {comPortVar}")
		
		current_port = comPortVar
	
		read_serial_data()
		root.after(100, read_serial_data)

	except serial.SerialException as e:
		print(f"Failed to open COM port {comPortVar}; {e}")
		tk.messagebox.showerror("COM Port Error", f"Cannot open {comPortVar}: {e}")

	except Exception as e:
		print(f"Unexpected error: {e}")
		tk.messagebox.showerror("Unexpected Error :", f" {e}")

def close_port():
	try:
		if ser.is_open:
			ser.close()
			print("COM port closed.")
	except Exception as e:
		print("Error closing COM Port.")

for index, onePort in enumerate(ports):
		comButton = tk.Button(COM_frame, text=onePort, width=50, command = functools.partial(initComPort, index = ports.index(onePort)))
		comButton.grid(row=ports.index(onePort),column=7, rowspan=1)
closeButton = tk.Button(COM_frame, text = "Close Port", command = close_port).grid(row=len(ports), column=7)



## GRAPH PLOTTING AND SERIAL DATA

time_data = []
distance_data = []
setpoint_data = []
voltage_data = []

fig_1 = plt.Figure()
ax = fig_1.add_subplot(111)
line1, = ax.plot([], [], label ='h(t)', color='blueviolet')
ax.set_xlabel('time / s')
line2, = ax.plot([],[], label = "h$_S$$_P$(t)", color='black', linestyle="--")
ax.set_ylabel('height / cm')


ax.set_xlim(0, 60)
ax.set_ylim(0,101)
ax.set_title("actual and setpoint distance vs time")
ax.legend()

distance_canvas = FigureCanvasTkAgg(fig_1, master=plot_frame)
distance_canvas.get_tk_widget().grid(row=0, column=0, rowspan=10, columnspan=7)


fig_2 = plt.Figure()
ay = fig_2.add_subplot(111)
line3, = ay.plot([],[],label = "V(t)", color='orangered')
ay.set_ylabel('V(t) / V')
ay.set_xlabel('time / s')

ay.set_xlim(0, 60)
ay.set_ylim(5,13) #Adjust voltage according to your power supply settings 
ay.set_title("voltage vs time")
ay.legend()


voltage_canvas = FigureCanvasTkAgg(fig_2, master=plot_frame)
voltage_canvas.get_tk_widget().grid(row=0, column=10, rowspan=10, columnspan=7)

def update_plot():

	if len(time_data) > 0:
		current_time = time_data[-1]

	if current_time > 60:
		ax.set_xlim(current_time-60, current_time)
		ay.set_xlim(current_time-60, current_time)
	else:
		ax.set_xlim(0,60)
		ay.set_xlim(0,60)

	line1.set_xdata(time_data)
	line1.set_ydata(distance_data)

	line2.set_xdata(time_data)
	line2.set_ydata(setpoint_data)

	line3.set_xdata(time_data)
	line3.set_ydata(voltage_data)

	ax.relim()
	ax.autoscale_view(True, True, False)
	distance_canvas.draw()

	ay.relim()
	ay.autoscale_view(True, True, False)
	voltage_canvas.draw()

def read_serial_data():
	try:
		if ser.is_open and ser.in_waiting > 0:
			line = ser.readline().decode("utf-8").strip()
	
			currsec, Distance, Setpoint, Output = map(float, line.split (";"))

			time_data.append(currsec)
			distance_data.append(Distance)
			setpoint_data.append(Setpoint)
			voltage_data.append(Output)

			time_data[:]=time_data[-600:]
			distance_data[:]=distance_data[-600:]
			setpoint_data[:]=setpoint_data[-600:]
			voltage_data[:]=voltage_data[-600:]

			update_plot()

			
			serial_text.insert(tk.END, f"{currsec:2f};{Distance:.2f};{Setpoint:.2f};{Output:.2f}\n")

			serial_text.see(tk.END)


	except Exception as e:
		print(f"Error reading data: {e}")
	
	root.after(10, read_serial_data)

root.after(100, read_serial_data)

def clear_plot():
	time_data.clear()
	distance_data.clear()
	setpoint_data.clear()
	voltage_data.clear()

	line1.set_xdata([])
	line1.set_ydata([])
	line2.set_xdata([])
	line2.set_ydata([])
	line3.set_xdata([])
	line3.set_ydata([])

	ax.set_xlim(0,60)
	ax.set_ylim(0,101)
	
	ay.set_xlim(0,60)
	ay.set_ylim(5,13)

	distance_canvas.draw()
	voltage_canvas.draw()

def clear_serial():

	serial_text.delete(1.0, tk.END)

def send_command(command):		

		command_with_terminator = command + "\n"+"\r"
		ser.write(command_with_terminator.encode())
		ser.flush()
		print(f"Command Sent: {command_with_terminator.strip()}")
		time.sleep(0.1)


##DATA SAVING 

def save_file():
	filename = filedialog.asksaveasfilename(defaultextension='.csv',
						filetypes=[("CSV files","*.csv"), ("All files","*.*")],
						title="Choose filename")
	if filename:
		try:
			data = serial_text.get("1.0", tk.END).strip()
			data= data.replace(";",",")
			with open(filename, 'w') as file:
				file.write("Time (s), Distance (cm), Setpoint (cm), Voltage(V)\n")
				file.write(data)
			print(f"Data saved to {filename}")
		except Exception as e:
			print(f"Error saving file: {e}")
	else:
		print("Save operation was cancelled")

save_button = tk.Button(serial_frame, text="SAVE DATA", command=save_file,bg="sky blue", fg="white", width =10, font=('Arial',10))
save_button.grid(row=3, column=34, columnspan=2, padx=10)


save_button = tk.Button(plot_frame, text="SAVE DATA", command=save_file,bg="sky blue", fg="white",width =15, font=('Arial',10))
save_button.grid(row=10, column=4, rowspan=3, columnspan=2, pady=10)



##COMMANDS

def update_baselineHeight():

	baselineHeight_value = baselineHeight_slider.get()
	send_command(f"@BH {baselineHeight_value}")


def update_amplitude():

	amplitude_value = amplitude_slider.get()
	send_command(f'@AMP {amplitude_value}')


def update_period():

	period_value = period_slider.get()
	send_command(f'@TP {period_value}')


def update_Kp(): 

	Kp_value = Kp_entry.get()
	send_command(f'@KP {Kp_value}')

def update_Ki(): 

	Ki_value = Ki_entry.get()
	send_command(f'@KI {Ki_value}')

def update_Kd(): 

	Kd_value = Kd_entry.get()
	send_command(f'@KD {Kd_value}')


def go_control():
	
	global system_running
	if not ser.is_open:
		tk.messagebox.showwarning("COM port not open"," Please select a COM port first")
		return
	if system_running:
		tk.messagebox.showwarning("Already Running", "Press STOP before pressing GO again")
		return

	system_running = True

	clear_plot()
	clear_serial()

	baselineHeight_value = baselineHeight_slider.get()
	send_command(f'@BH {baselineHeight_value}')

	amplitude_value = amplitude_slider.get()
	send_command(f'@AMP {amplitude_value}')

	period_value = period_slider.get()
	send_command(f'@TP {period_value}')

	Kp_value = Kp_entry.get()
	send_command(f' @KP {Kp_value}')

	Ki_value = Ki_entry.get()
	send_command(f' @KI {Ki_value}')
	
	Kd_value = Kd_entry.get()
	send_command(f' @KD {Kd_value}')
	
	send_command("@GO")
	
def stop_control():
	global system_running
	send_command("@STOP")
	system_running = False
	time.sleep(0.5)
	ser.flushInput()
	ser.flushOutput()
	baselineHeight_value = baselineHeight_slider.get()
	amplitude_value = amplitude_slider.get()
	period_value = period_slider.get()
	Kp_value = Kp_entry.get()
	Ki_value = Ki_entry.get()
	Kd_value = Kd_entry.get()
	serial_text.insert(tk.END, f" Kp: {Kp_value};Ki: {Ki_value};Kd: {Kd_value}; Baseline Height: {baselineHeight_value}; Amplitude:{amplitude_value};Time Period: {period_value}\n")
	serial_text.see(tk.END)
	



##INTERFACE CONTROLS

baselineHeight_slider = tk.Scale(main_frame, from_=100, to=0,resolution=1, bg="coral", fg="black", length=200)
baselineHeight_slider.set("80")
baselineHeight_slider.grid(row=0, column=0, rowspan=10)

amplitude_slider = tk.Scale(main_frame, from_=40, to=0, resolution=1, bg="teal", fg="black", length=200)
amplitude_slider.set("0")
amplitude_slider.grid(row=0, column=2, rowspan=10)

period_slider = tk.Scale(main_frame, from_=30, to=0, resolution=1, bg="palevioletred", fg="black",length=200)
period_slider.set("10")
period_slider.grid(row=0, column=4, rowspan=10)

update_baselineHeight_button = tk.Button(main_frame, text="Update\nBaseline Height", command=update_baselineHeight, width =15)
update_baselineHeight_button.grid(row=3, column=1,rowspan=2, padx=2, sticky="w")

update_amplitude_button = tk.Button(main_frame, text="Update\n Amplitude", command=update_amplitude, width =15)
update_amplitude_button.grid(row=3, column=3, rowspan=2,padx=2, sticky="w")

update_period_button = tk.Button(main_frame, text="Update\n Period", command=update_period, width =15)
update_period_button.grid(row=3, column=5, rowspan=2,padx=2, sticky="w")

tk.Button(plot_frame, text="GO", command=go_control, bg="seagreen", fg="white",width =15, font=('Arial',10)).grid(row=10, column=0, rowspan=3, columnspan=2, pady=10)

tk.Button(plot_frame, text="STOP", command=stop_control, bg="firebrick", fg="white",width =15, font=('Arial',10)).grid(row=10, column=2, rowspan=3, columnspan=2, pady=10)

tk.Label(main_frame, text="Kp:",width=2).grid(row=12, column=0, sticky="e")
Kp_entry = tk.Entry(main_frame, width=5)
Kp_entry.insert(0,"0.05")
Kp_entry.grid(row=12, column=1)

update_Kp_button = tk.Button(main_frame, text="Update Kp", command=update_Kp, width =10)
update_Kp_button.grid(row=12, column=2,columnspan=2, pady=10, sticky="w")

tk.Label(main_frame, text="Ki:",width=2).grid(row=13, column=0, sticky="e")
Ki_entry = tk.Entry(main_frame, width=5)
Ki_entry.insert(0,"0.05")
Ki_entry.grid(row=13, column=1)


update_Ki_button = tk.Button(main_frame, text="Update Ki", command=update_Ki, width =10)
update_Ki_button.grid(row=13, column=2,columnspan=2, pady=10, sticky="w")


tk.Label(main_frame, text="Kd:",width=2).grid(row=14, column=0, sticky="e")
Kd_entry = tk.Entry(main_frame, width=5)
Kd_entry.insert(0,"0.002")
Kd_entry.grid(row=14, column=1)

update_Kd_button = tk.Button(main_frame, text="Update Kd", command=update_Kd, width =10)
update_Kd_button.grid(row=14, column=2, columnspan=2,pady=10, sticky="w")

tk.Button(serial_frame, text="GO", command=go_control, bg="seagreen", fg="white",width =10, font=('Arial',10)).grid(row=1, column=33, columnspan=2, padx=10)

tk.Button(serial_frame, text="STOP", command=stop_control, bg="firebrick", fg="white",width =10, font=('Arial',10)).grid(row=1, column=35, columnspan=2, padx=10)

root.mainloop()
ser.close()



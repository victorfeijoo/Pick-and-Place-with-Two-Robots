import tkinter as tk
from PIL import Image,ImageTk
from std_msgs.msg import String, Int8
import rclpy
from rclpy.node import Node
import threading
from threading import Thread
import asyncio



class interfaz_menu(Node):
    def __init__(self):
        super().__init__("interfaz_menu")

        #Publishers for communications

        self.publisher_resp = self.create_publisher(Int8, "resp_aplicacion", 10)
        self.publisher_color = self.create_publisher(String, "sec_color", 10)

        #Values to be communicated

        self.respuesta = 0
        self.sec_color = String()

        #Tkinter instance
        self.ventana = tk.Tk()
        self.ventana.title("Pick and Place con UR3e y visión") 
        self.ventana.configure(bg="#FFFFFF") 

        #Images load
        self.robot_imagen = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/robot.jpeg")
        self.robot_imagen = self.robot_imagen.resize((400, 488))
        self.imagen_tk_robot = ImageTk.PhotoImage(self.robot_imagen)

        self.imagen_v = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/verde.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_v = self.imagen_v.resize((10000, 4000)) 
        self.imagen_tk_v = ImageTk.PhotoImage(self.imagen_v)

        self.imagen_a = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/azul.jpeg")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_a = self.imagen_a.resize((10000, 4000)) 
        self.imagen_tk_a = ImageTk.PhotoImage(self.imagen_a)

        self.imagen_n = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/naranja.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_n = self.imagen_n.resize((10000, 4000)) 
        self.imagen_tk_n = ImageTk.PhotoImage(self.imagen_n)

        self.imagen_m = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/morado.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_m = self.imagen_m.resize((10000, 4000)) 
        self.imagen_tk_m = ImageTk.PhotoImage(self.imagen_m)

        self.imagen_ap1 = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/AP1.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_ap1 = self.imagen_ap1.resize((200, 140)) 
        self.imagen_tk_ap1 = ImageTk.PhotoImage(self.imagen_ap1)

        self.imagen_ap2 = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/AP2.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_ap2 = self.imagen_ap2.resize((200, 140)) 
        self.imagen_tk_ap2 = ImageTk.PhotoImage(self.imagen_ap2)

        self.imagen_ap3 = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/AP3.png") 
        self.imagen_ap3 = self.imagen_ap3.resize((200, 140)) 
        self.imagen_tk_ap3 = ImageTk.PhotoImage(self.imagen_ap3)

        #Window size
        self.ventana.geometry("1370x500") 

        #Widgets
        self.etiqueta_robot = tk.Label(self.ventana, image=self.imagen_tk_robot)
        self.etiqueta_robot.place(x= 5, y = 5)

        self.etiqueta0 = tk.Label(self.ventana, text="        BIENVENIDO        ",font=("Arial", 18, "bold"),bg="#333333",fg="white")
        self.etiqueta0.place(x=780, y=10)

        self.etiqueta1 = tk.Label(self.ventana, text="       A continuación, va a poder elegir entre diferentes aplicaciones de PICK & PLACE.      ",font=("Arial", 15, "bold"),bg="#333333",fg="white")
        self.etiqueta1.place(x=469, y=50)

        self.etiqueta_color = tk.Label(self.ventana, text="     Seleccione los colores por orden de las piezas a recoger.     ",bg="#333333",fg="white",font=("Arial", 12, "bold"))
        self.etiqueta_color.place(x=469, y=140)

        self.etiqueta_color_2 = tk.Label(self.ventana, text="     ¡Haga un click por color!                                                      ----->    ",bg="#333333",fg="white",font=("Arial", 12, "bold"))
        self.etiqueta_color_2.place(x=469, y=170)

        self.etiqueta_ap1 = tk.Label(self.ventana, text="  1. Orden de piezas  ",bg="#333333",fg="white",font=("Arial", 12, "bold"))
        self.etiqueta_ap1.place(x=508, y=455)

        self.etiqueta_ap2= tk.Label(self.ventana, text="  2. Paletizado de piezas  ",bg="#333333",fg="white",font=("Arial", 12, "bold"))
        self.etiqueta_ap2.place(x=810, y=455)

        self.etiqueta_ap3= tk.Label(self.ventana, text="  3. Despaletizado de piezas  ",bg="#333333",fg="white",font=("Arial", 12, "bold"))
        self.etiqueta_ap3.place(x=1120, y=455)

        self.etiqueta_op= tk.Label(self.ventana, text="     Seleccione la aplicación que desea llevar a cabo:     ",bg="#333333",fg="white",font=("Arial", 12, "bold"))
        self.etiqueta_op.place(x=469, y=255)

        #Buttons
        self.boton1 = tk.Button(self.ventana, image=self.imagen_tk_ap1, width=200, height=140, command=self.respuesta_1)
        self.boton2 = tk.Button(self.ventana, image=self.imagen_tk_ap2, width=200, height=140, command=self.respuesta_2)
        self.boton3 = tk.Button(self.ventana, image=self.imagen_tk_ap3, width=200, height=140, command=self.respuesta_3)
        self.boton4 = tk.Button(self.ventana, image=self.imagen_tk_n, width=100, height=70, command=self.color_naranja)
        self.boton5 = tk.Button(self.ventana, image=self.imagen_tk_m, width=100, height=70,command=self.color_morado)
        self.boton6 = tk.Button(self.ventana, image=self.imagen_tk_v, width=100, height=70,command=self.color_verde)
        self.boton7 = tk.Button(self.ventana,  image=self.imagen_tk_a, width=100, height=70,command=self.color_azul)
        
        #Buttons positioning
        self.boton1.place(x=483, y=300)  
        self.boton2.place(x=800, y=300) 
        self.boton3.place(x=1127, y=300)  
        self.boton4.place(x=1117, y=95)  
        self.boton5.place(x=1227, y=95) 
        self.boton6.place(x=1227, y=175) 
        self.boton7.place(x=1117, y=175) 

        #Window loop
        self.ventana.mainloop()

    #Additional function for the publisher to publish the correct data
    def respuesta_1(self):
        msg = Int8()
        msg.data = 1
        self.publisher_resp.publish(msg)

    def respuesta_2(self):
        msg = Int8()
        msg.data = 2
        self.publisher_resp.publish(msg)

    def respuesta_3(self):
        msg = Int8()
        msg.data = 3
        self.publisher_resp.publish(msg)

    def color_naranja(self):
        self.sec_color.data += "n"
        if len(self.sec_color.data) == 4:
            self.publisher_color.publish(self.sec_color)

    def color_morado(self):
        self.sec_color.data += "m"
        if len(self.sec_color.data) == 4:
            self.publisher_color.publish(self.sec_color)

    def color_verde(self):
        self.sec_color.data += "v"
        if len(self.sec_color.data) == 4:
            self.publisher_color.publish(self.sec_color)

    def color_azul(self):
        self.sec_color.data += "a"
        if len(self.sec_color.data) == 4:
            self.publisher_color.publish(self.sec_color)

def main(args=None):
    rclpy.init(args=args)
    interfaz = interfaz_menu() #Object of the class

    while True:
        rclpy.spin_once(interfaz)

    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
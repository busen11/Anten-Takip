import socket #TCP/IP veri iletimi için.
import cv2 #OpenCV(Görüntü İşleme) kütüphanesi.
import pickle #Veri serileştirme ve desirileştirme işlemleri için.
import struct #Veri paketleme ve açma işlemleri için.
from threading import Thread #Aynı anda birden fazla işlem çalıştırmak için.
from pymavlink import mavutil #Pixhawk’tan veri almak için gerekli MAVLink haberleşmesini sağlar.
import time #Zaman
import serial #Arduino ile seri iletişim kurmak için.
from math import sqrt,radians,degrees,cos,sin,atan2,asin,acos,atan #Açı ve mesafe hesaplamaları için.
#import RPi.GPIO as GPIO #Raspberry Pi'ın GPIO (General Purpose Input/Output - Genel Amaçlı Giriş/Çıkış) pinlerini GPIO olarak kullanmak için.
import numpy as np #Matematiksel işlemler için.

class anten:
    # GPS Verisi Okuma
    def gps(self):
        gps = serial.Serial('COM3', 9600)  # GPS cihazının bağlı olduğu seri portu belirtin
        while True:
            # GPS verisini al
            gps_data = gps.readline().decode("utf-8").strip() 
            # GPS verisini işleme
            print("GPS Verisi Alindi:", gps_data)
            time.sleep(1)

    def __init__(self):
        self.servoPIN = 11 #Raspberry Pi’de hangi GPIO pinine servo motor bağladıysan, o numarayı burada yaz.
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(servoPIN, GPIO.OUT)
        #self.pwm = GPIO.PWM(self.servoPIN, 50)
        #self.pwm.start(0)
        self.yer_istasyonu = {"lat":-35.3632604,"lon":149.1652399,"alt":1,"heading":33}   #Latitude(Enlem) #Longitude(Boylam) #Altitude(Rakım) #Heading(Baş Yönü)
        self.aci=90 #Bu, servonun(ve dolayısıyla antenin) başlangıçta 90 dereceye bakması için.
        #self.servo(self.aci)
        self.tcp_ip = "10.15.184.117"  # TCP sunucu IP
        self.tcp_port = 9999       # TCP sunucu port
        self.udp_ip = "10.15.184.117"  # UDP hedef IP
        self.udp_port = 9999       # UDP hedef port
        # Pixhawk bağlantısı
        self.vehicle = mavutil.mavlink_connection('udp:10.15.184.117')
        self.vehicle.wait_heartbeat()
        print("Pixhawk bagli!")

        # Arduino bağlantısı (COM portunu sistemine göre değiştir)
        self.arduino = serial.Serial('COM3', 9600)
        print("Arduino bagli!")

    def servo(self,aci):
        self.aci -= aci
        x=(1/90)*self.aci + 1
        duty=x*33
        print("aci =",self.aci,"duty =",duty)
        #self.pwm.ChangeDutyCycle(duty)
        """
              self.pwm.stop()
              GPIO.cleanup()
        """
    def haversine(self, lat1, lon1, lat2, lon2):
        # Radyan cinsinden koordinatları al
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        # Haversine formülü
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        # Dünya yarıçapı (ortalama)
        R = 6371.0  # Dünya yarıçapı kilometre cinsinden

        # Gerçek uzaklık hesaplama
        distance = R * c * 1000  # Metre cinsinden

        return distance

    def hesapla(self,lat,lon,alt): # aracın konum bilgileri
        #Yer İstasyonu konumu
        
        lat_u = self.haversine(lat, self.yer_istasyonu["lon"], self.yer_istasyonu["lat"], self.yer_istasyonu["lon"])
        lon_u = self.haversine(self.yer_istasyonu["lat"], lon, self.yer_istasyonu["lat"], self.yer_istasyonu["lon"])
        #self.yer_istasyonu["heading"] %=360
        if int(lat_u) == 0 and int(lon_u) == 0:
            yatay_aci = 0
        elif int(lat_u) == 0:
            if self.yer_istasyonu["lon"] > lon:
                if abs(90 - self.yer_istasyonu["heading"] )>abs(self.yer_istasyonu["heading"]-90):
                    yatay_aci = 90 - self.yer_istasyonu["heading"]
                else:
                    yatay_aci = self.yer_istasyonu["heading"] -90
                self.yer_istasyonu["heading"] = 90
            else:
                if abs(270 - self.yer_istasyonu["heading"] )>abs(self.yer_istasyonu["heading"]-270):
                    yatay_aci = 270 - self.yer_istasyonu["heading"]
                else:
                    yatay_aci = self.yer_istasyonu["heading"] -270
                self.yer_istasyonu["heading"] = 270       
            return yatay_aci
        
        elif int(lon_u) == 0:
            if self.yer_istasyonu["lat"] > lat:
                if abs(360 - self.yer_istasyonu["heading"] )>abs(self.yer_istasyonu["heading"]):
                    yatay_aci = 360 - self.yer_istasyonu["heading"]
                else:
                    yatay_aci = self.yer_istasyonu["heading"] - 360
                self.yer_istasyonu["heading"] = 0       
                
            else:
                if abs(180 - self.yer_istasyonu["heading"] )>abs(self.yer_istasyonu["heading"]-180):
                    yatay_aci = 180 - self.yer_istasyonu["heading"]
                else:
                    yatay_aci = self.yer_istasyonu["heading"] -180
                self.yer_istasyonu["heading"] = 180       
            return  yatay_aci
        
        else:
            yatay_aci = degrees(atan(lon_u/lat_u))
            hipotenüs = sqrt(pow(lat_u,2)+pow(lon_u,2))
            dikey_aci = degrees(atan((alt-self.yer_istasyonu["alt"])/hipotenüs))
            print("hesaplanan",yatay_aci)
            k1,k2=1,1
            if 0<=self.yer_istasyonu["heading"]<90:
                if lon < self.yer_istasyonu["lon"]:
                    k1=-1
                if lat < self.yer_istasyonu["lat"]:
                    k2=-1
                if k1+k2 <=0:
                    yatay_aci *=-1
                else:
                    yatay_aci*=-1
                    yatay_aci +=180
          
                yatay_aci -= self.yer_istasyonu["heading"]
                self.yer_istasyonu["heading"] +=  yatay_aci
                self.yer_istasyonu["heading"] %= 360
                print("işlem sonrası",yatay_aci,self.yer_istasyonu["heading"])

            elif 90<=self.yer_istasyonu["heading"]<180:
                if lon < self.yer_istasyonu["lon"]:
                    k1 = -1
                if abs(lat) < abs(self.yer_istasyonu["lat"]):
                    k2=-1

                if k1>0 and k2<0:
                    yatay_aci -= self.yer_istasyonu["heading"]
                    
                else:
                    if k1<0 and k2>0:
                        pass
                    else:
                        yatay_aci *= -1
                    yatay_aci +=180 -self.yer_istasyonu["heading"]
                self.yer_istasyonu["heading"] +=  yatay_aci
                self.yer_istasyonu["heading"] %= 360
                print("işlem sonrası",yatay_aci,self.yer_istasyonu["heading"])

            elif 180<=self.yer_istasyonu["heading"]<270:
                if lon < self.yer_istasyonu["lon"]:
                    k1 = -1
                if abs(lat) < abs(self.yer_istasyonu["lat"]):
                    k2=-1
                if k1+k2 == 0:
                    yatay_aci += 180-self.yer_istasyonu["heading"]
                elif k1+k2<0:
                    yatay_aci *=-1
                    yatay_aci += 360-self.yer_istasyonu["heading"]
                else:
                    yatay_aci += self.yer_istasyonu["heading"] -180
                    yatay_aci *=-1

                self.yer_istasyonu["heading"] +=  yatay_aci
                self.yer_istasyonu["heading"] %= 360

            else:
                if lon < self.yer_istasyonu["lon"]:
                    k1 = -1
                if abs(lat) < abs(self.yer_istasyonu["lat"]):
                    k2=-1
                    
                if k1>0 and k2<0:
                    yatay_aci += 360-self.yer_istasyonu["heading"]

                elif k1<0 and k2>0:
                    yatay_aci +=180-self.yer_istasyonu["heading"]

                else:
                    yatay_aci -= 360
                    yatay_aci *= -1
                    yatay_aci -= self.yer_istasyonu["heading"]
                self.yer_istasyonu["heading"] +=  yatay_aci
                self.yer_istasyonu["heading"] %= 360
            

        return yatay_aci
    
    def gonder(self):
        vehicle= mavutil.mavlink_connection('tcp:10.15.184.117', baud=115200)
        """s = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM
        )  # Socket will create with TCP and IP protocols
        # This method will bind the sockets with server and port no
        s.bind(("192.168.1.167", 9999))
        s.listen(1)  # Will allow a maximum of one connection to the socket
        c, addr = s.accept()  # will wait for the client to accept the connection"""

        """host_ip = '10.15.168.26'  # paste your server ip address here
        veri = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        port = 9996
        veri.bind((host_ip, port))
        veri.listen(1)
        print("bağlantı bekleniyor")
        client,add = veri.accept()
        print(add)"""
        while True:
            loc = str(vehicle.location(True))
            loc = loc.split(",")
            a=[]
            for l in loc:
                l = l.split("=")
                a.append(float(l[1]))
            #message = {"lat":a[0],"lon":a[1],"alt":a[2]}
            #message = pickle.dumps(message)

            #c.send(message)
            #time.sleep(0.05)
            aci = self.hesapla(-35.3662357,149.1670632,a[2])
            #print(aci)
            #self.servo(aci)
            #message = {"aci":aci}
            #data = pickle.dumps(message)
            #client.send(data)
            time.sleep(1)

#anten=anten()
#anten.gonder()
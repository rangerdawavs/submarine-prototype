import serial

port = serial.Serial("/dev/rfcomm"+input("port number"), baudrate=9600)
port.write(input("primer coeficiente").encode('utf-8'))
port.write(input("segundo coeficiente").encode('utf-8'))
port.write(input("tercer coeficiente").encode('utf-8'))
port.write(input("cuarto coeficiente").encode('utf-8'))
while True:
    ceta = input('current angle of robot')
    ceta2 = input('angle of robot to center')
    mag = input('distance to center')
    print(ceta,",",ceta2,",",mag)
    port.write(str(int(ceta)).encode('utf-8'))
    port.write((",").encode('utf-8'))
    port.write(str(int(ceta2)).encode('utf-8'))
    port.write((",").encode('utf-8'))
    port.write(str(int(mag)).encode('utf-8'))
    #port.write((";").encode('utf-8'))
    print("sending")

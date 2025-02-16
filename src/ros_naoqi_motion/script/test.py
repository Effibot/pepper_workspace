#! /usr/bin/python
import socket


def client_program():
    host = "192.168.130.103"  # as both code is running on same pc
    port = 9999  # socket server port number
    try:
        client_socket = socket.socket()  # instantiate
        client_socket.connect((host, port))  # connect to the server

        message = input(" -> ")  # take input

        while message.lower().strip() != "bye":
            client_socket.send(message.encode())  # send message
            data = client_socket.recv(1024).decode()  # receive response
            print("Received from server: " + data)  # show in terminal
            if data == "JOB_DONE":
                client_socket.close()
                break
            else:
                message = input(" -> ")  # again take input
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt (SIGINT)")
        client_socket.close()  # close the connection


if __name__ == "__main__":
    client_program()
    print("Client closed")

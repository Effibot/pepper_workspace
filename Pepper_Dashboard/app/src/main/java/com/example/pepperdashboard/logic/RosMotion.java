package com.example.pepperdashboard.logic;
import android.content.Context;
import android.content.res.AssetManager;
import android.os.AsyncTask;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
public class RosMotion  extends AsyncTask<String, Void, Void>{
    private Socket clientSocket;
    private PrintWriter out;
    private BufferedReader in;
    private String ip;
    private int port;
    private Context context;
    private String content;
    private ExecutorService executor;
    private AssetManager assetManager;

    private static final String MOVE_ARM_UP = "move_arm_up";
    private static final String MOVE_ARM_DOWN = "move_arm_down";

    public RosMotion(String ip, int port, Context context, AssetManager assetManager) {
        super();
        this.ip = ip;
        this.port = port;
        this.context = context;
        this.content = "";
        this.executor = Executors.newSingleThreadExecutor();
        this.assetManager = assetManager;
    }
    public String get_ip(){
        return this.ip;
    }

    public int getPort() {
        return port;
    }

    public void set_ip(String ip){
        this.ip = ip;
    }
    public void set_port(int port){
        this.port = port;
    }
    public void setContent(String string) {
        this.content = string;
    }

    public String get_arm_down() {
        return MOVE_ARM_DOWN;
    }
    public String get_arm_up() {
        return MOVE_ARM_UP;
    }

    public String sendMessage(String msg) {
        try {
            this.out.write(msg);
            this.out.flush();
            return in.readLine(); // blocking call that waits for the server to send a response
        } catch (Exception e) {
            System.out.println("Error in sendMessage: " + e.getMessage());
            return null;
        }
    }
    public void stopConnection() {
        // close the connection
        try{
            in.close();
            out.close();
            clientSocket.close();
        }catch(Exception e) {
            System.out.println("Error in stopConnection: " + e.getMessage());
        }
    }
    @Override
    protected Void doInBackground(String... params){
        executor.execute(()->{
            try {
                // check if the connection is already open
                this.clientSocket = new Socket(this.ip, this.port);
                this.out = new PrintWriter(new OutputStreamWriter(clientSocket.getOutputStream()));
                this.in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                // send the text to the ROS2 TTS service through the handler
                do_work();
                // close the connection
                // wait 1 second before closing the connection just to be sure that the message has been sent
                Thread.sleep(1000);
                in.close();
                out.close();
                clientSocket.close();
            } catch (Exception e) {
                System.out.println("Error in doInBackground: " + e.getMessage());
            }
        });
        return null;
    }

    private void do_work() {
        // when called at the start of the application, the robot will move its arm up
        boolean error = false;
        if (this.content.equals(MOVE_ARM_UP)) {
            String response = sendMessage("move_arm_up");
            if (!response.equals("ACK")){
                Toast.makeText(this.context, "Error in sending the message", Toast.LENGTH_LONG).show();
                error = true;
            }
        } else if (this.content.equals(MOVE_ARM_DOWN)) {
            String response = sendMessage("move_arm_down");
            if (!response.equals("ACK")){
                Toast.makeText(this.context, "Error in sending the message", Toast.LENGTH_LONG).show();
                error = true;
            }
        }
        if (error) {
            stopConnection();
        }

    }
}

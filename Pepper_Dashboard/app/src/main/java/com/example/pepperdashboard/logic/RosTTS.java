package com.example.pepperdashboard.logic;

/*
 * A simple TCP client that sends a message to the server and receives a response.
 * The client is implemented as a console application, while the server is implemented
 * as a ROS2 node which executes the TTS (text-to-speech) service.
 */

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

public class RosTTS extends AsyncTask<String, Void, Void>{
    private Socket clientSocket;
    private PrintWriter out;
    private BufferedReader in;
    private String ip;
    private int port;
    private Context context;
    private String content;
    private ExecutorService executor;
    private AssetManager assetManager;

    public RosTTS(String ip, int port, Context context, AssetManager assetManager) {
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


    public void setContent(String string) {
        this.content = string;
    }

    public void do_work() {
        // send the text to the ROS2 TTS service
        String[] storyContent = this.content.split("\n");
        boolean error = false;
        for(String line : storyContent){
            String response = sendMessage(line+"\n");
            if (!response.equals("ACK")){
                Toast.makeText(this.context, "Error in sending the message", Toast.LENGTH_LONG).show();
                error = true;
                break;
            }
        }
        // send the "JOB_DONE" message to the ROS2 TTS service
        if(!error){
            String response = sendMessage("JOB_DONE\n");
            Toast.makeText(this.context, response.toLowerCase(), Toast.LENGTH_LONG).show();
            stopConnection();
        }
    }

    public String getText(String file_name){
        try{
            // Open an InputStream to the asset representing the story
            InputStream story = assetManager.open(file_name+".txt");

            // Create a BufferedReader object to read lines from the story
            BufferedReader reader = new BufferedReader(new InputStreamReader(story));
            // Read the story while generating a new string that contains its content
            StringBuilder stringBuilder = new StringBuilder();
            String line;
            while((line = reader.readLine()) != null){
                stringBuilder.append(line);
                stringBuilder.append('\n');
            }
            // Close the reader and return the story
            reader.close();
            // set this object's content
            content = stringBuilder.toString();
            setContent(content);
            return content;

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

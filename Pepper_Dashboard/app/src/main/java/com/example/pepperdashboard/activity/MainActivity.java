package com.example.pepperdashboard.activity;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;


import com.example.pepperdashboard.R;
import com.example.pepperdashboard.device.iHealth;
import com.google.android.material.textfield.TextInputLayout;
public class MainActivity extends AppCompatActivity {

    private final String TAG = this.getClass().getSimpleName();
    private Button secondActivitytrigger;
    private TextInputLayout hostname;
    private TextInputLayout hostname_pepper;
    private iHealth iHealth;


    /**
     * The onCreate function is called when the activity is created.
     * It sets up the layout of the activity and initializes all variables.
     *
     *
     * @param Bundle savedInstanceState Save the state of the activity when it is destroyed
     *
     * @return Void
     *
     * @docauthor Trelent
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_ip);
        iHealth = new iHealth();
        secondActivitytrigger = findViewById(R.id.confirmBtn);
        secondActivitytrigger.setOnClickListener(new View.OnClickListener() {
            /**
             * The onClick function is called when the user clicks on the button.
             * It checks if a valid IP address has been entered, and if so it switches to the next activity.

             *
             * @param View view Get the context of the activity
                        private void switchactivities(string host_text) {
                            intent intent = new intent(view
             *
             * @return void
             *
             * @docauthor Lorenzo Rossi
             */
            @Override
            public void onClick(View view) {
                hostname = findViewById(R.id.textInputLayout);
                String host_text = String.valueOf(hostname.getEditText().getText());
                // Check if the user has entered a hostname: if the text is empty or doesn't match xxx.xxx.xxx give error
                if (host_text.isEmpty() || !host_text.matches("^(?:[0-9]{1,3}\\.){3}[0-9]{1,3}$")) {
                    hostname.setError("Please enter a valid IP address");
                    return;
                }
                hostname_pepper = findViewById(R.id.textInputLayoutPepper);
                String host_pepper_text = String.valueOf(hostname_pepper.getEditText().getText());
                // Check if the user has entered a hostname: if the text is empty or doesn't match xxx.xxx.xxx give error
                if (host_pepper_text.isEmpty() || !host_pepper_text.matches("^(?:[0-9]{1,3}\\.){3}[0-9]{1,3}$")) {
                    hostname_pepper.setError("Please enter a valid IP address");
                    return;
                }
//                new Thread(new Runnable() {
//                    @Override
//                    public void run() {
//                        JSch jsch = new JSch();
//                        try {
//                            Session session = jsch.getSession("nao",host_pepper_text,22);
//                            session.setPassword("pepshtpsw00n");
//                            Properties prop = new Properties();
//                            prop.put("StrictHostKeyChecking", "no");
//                            session.setConfig(prop);
//
//                            String command = "qicli call ALTextToSpeech.say \"Hello, I am Pepper!\"";
//                            session.connect();
//                            ChannelExec channel = (ChannelExec) session.openChannel("exec");
//                            channel.setCommand(command);
//
//                        } catch (JSchException e) {
//                            throw new RuntimeException(e);
//                        }
//                    }
//                }).start();
                switchActivities(host_text, host_pepper_text);

            }


        });

        checkPermission();
//        QiSDK.register(this, this);

    }

    /**
     * The onDestroy function is called when the activity is destroyed.
     * This function unregisters the QiSDK from this activity.

     *
     *
     * @return Nothing
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    protected void onDestroy() {
        super.onDestroy();

        // Unregister the QiSDK
//        QiSDK.unregister(this, this);
    }


    /**
     * The checkPermission function checks to see if the user has granted permission for the app to access
     * coarse location, fine location, external storage, and audio recording. If not, it requests that they do so.

     *
     *
     * @return void
     *
     * @docauthor Lorenzo Rossi
     */
    private void checkPermission() {
        int version = Build.VERSION.SDK_INT;
        if (version > 30) {
            if (ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE,
                        Manifest.permission.ACCESS_COARSE_LOCATION,
                        Manifest.permission.ACCESS_FINE_LOCATION,
                        Manifest.permission.RECORD_AUDIO,
                        Manifest.permission.BLUETOOTH_SCAN,
                        Manifest.permission.BLUETOOTH_CONNECT}, 1);
            }
        } else {
            if (ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(iHealth.applicationContext, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.RECORD_AUDIO}, 1);
            }
        }

    }


    /**
     * The switchActivities function is called when the user clicks on the &quot;Connect&quot; button.
     * It takes in a String parameter, host_text, which is passed to it from the getHostText function.
     * The switchActivities function then creates an Intent object that will be used to switch activities and pass data between them.
     * The Intent object created by this function switches from MainActivity (the current activity) to MenuActivity (the next activity).

     *
     * @param String host_text Pass the host name to the next activity
     *
     * @return A new menuactivity
     *
     * @docauthor Trelent
     */
    private void switchActivities(String host_text, String host_pepper_text) {
        Intent switchActivityIntent = new Intent(this, MenuActivity.class);
        switchActivityIntent.putExtra("host", host_text);
        switchActivityIntent.putExtra("host_pepper", host_pepper_text);
        startActivity(switchActivityIntent);
    }


//    @Override
//    public void onRobotFocusGained(QiContext qiContext) {
//        Log.i(TAG, "Robot focus gained");
//        // The robot focus is gained. Say "Hello, you!".
//        // Create a new say action.
//        SayBuilder.with(qiContext) // Create the builder with the context.
//                .withText("Hello, you!") // Set the text to say.
//                .build() // Build the say action.
//                .run(); // Execute the action.
//    }
//
//    @Override
//    public void onRobotFocusLost() {
//        Log.e(TAG, "Robot focus lost");
//    }
//
//    @Override
//    public void onRobotFocusRefused(String reason) {
//        Log.e(TAG, "Robot focus refused: " + reason);
//    }
}

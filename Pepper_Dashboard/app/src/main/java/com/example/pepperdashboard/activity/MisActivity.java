package com.example.pepperdashboard.activity;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.ImageButton;

import androidx.appcompat.app.AppCompatActivity;

import com.example.pepperdashboard.R;
import com.example.pepperdashboard.device.PO3;
import com.example.pepperdashboard.model.DeviceCharacteristic;

import java.util.ArrayList;
import java.util.List;


public class MisActivity extends AppCompatActivity {
    private boolean is_authenticade;
    private static Context context;
    private ImageButton pulseButton;
    private List<DeviceCharacteristic> list_ScanDevices = new ArrayList<>();




    /**
     * The onCreate function is called when the activity is first created.
     * It sets up the layout of the activity and initializes all variables.

     *
     * @param Bundle savedInstanceState Pass data between activities
     *
     * @return A view
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.misuration_activiy);
        pulseButton = findViewById(R.id.pulseButton);
        pulseButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent switchActivityIntent = new Intent(MisActivity.this, PO3.class);
                switchActivityIntent.putExtra("host", getIntent().getStringExtra("host"));
                startActivity(switchActivityIntent);
            }
        });
    }

    /**
     * The onBackPressed function is a function that overrides the default back button functionality.
     * The onBackPressed function allows us to specify what happens when the user presses the back button.
     * In this case, we want to switch activities from MisActivity to MenuActivity when they press the back button.

     *
     *
     * @return A switchactivityintent
     *
     * @docauthor Lorenz Rossi
     */
    @Override
    public void onBackPressed() {
        super.onBackPressed();
        Intent switchActivityIntent = new Intent(MisActivity.this, MenuActivity.class);
        switchActivityIntent.putExtra("host", getIntent().getStringExtra("host"));
        startActivity(switchActivityIntent);
    }




}

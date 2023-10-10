package com.example.pepperdashboard.activity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.ImageButton;

import androidx.appcompat.app.AppCompatActivity;

import com.example.pepperdashboard.R;


public class SplashScreenActivity extends AppCompatActivity {

    private ImageButton settingButton;
    @Override
    public void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        setContentView(R.layout.enea_login);
        settingButton = (ImageButton) findViewById(R.id.settingButton);

        settingButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent switchActivityIntent = new Intent(SplashScreenActivity.this, MainActivity.class);
                startActivity(switchActivityIntent);
            }
        });
    }
}

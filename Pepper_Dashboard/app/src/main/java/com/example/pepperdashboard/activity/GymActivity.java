package com.example.pepperdashboard.activity;

import android.content.Intent;
import android.os.Bundle;

import androidx.appcompat.app.AppCompatActivity;

import com.example.pepperdashboard.R;

public class GymActivity extends AppCompatActivity {
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.gym_activity);
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        Intent switchActivityIntent = new Intent(this, MenuActivity.class);
        startActivity(switchActivityIntent);
    }
}

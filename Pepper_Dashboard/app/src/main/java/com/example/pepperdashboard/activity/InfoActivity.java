package com.example.pepperdashboard.activity;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.content.res.AssetManager;
import android.os.Bundle;
import android.widget.TextView;

import com.example.pepperdashboard.R;
import com.example.pepperdashboard.logic.RosTTS;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;


public class InfoActivity extends AppCompatActivity{
    private TextView infoText;
    private RosTTS tts;
    private AssetManager assetManager;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.info_activity);

        infoText = findViewById(R.id.infotextView);
        tts = new RosTTS(getIntent().getStringExtra("host"), 9090, this, getAssets());
        infoText.setText(tts.getText("info"));
        tts.execute();
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        Intent switchActivityIntent = new Intent(this, MenuActivity.class);
        switchActivityIntent.putExtra("host", getIntent().getStringExtra("host"));
        startActivity(switchActivityIntent);
    }

}

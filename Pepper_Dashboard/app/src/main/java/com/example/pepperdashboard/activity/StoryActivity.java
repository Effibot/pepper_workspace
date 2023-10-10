package com.example.pepperdashboard.activity;

import static android.os.SystemClock.sleep;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.content.res.AssetManager;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.example.pepperdashboard.R;
import com.example.pepperdashboard.logic.RosTTS;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Locale;
import java.util.Objects;

public class StoryActivity extends AppCompatActivity{
    private RosTTS tts;
    private TextView storyText;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.story_activity);
        storyText = findViewById(R.id.storyTextView);
        tts = new RosTTS(getIntent().getStringExtra("host"), 9090, this, getAssets());
        storyText.setText(tts.getText("story"));
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

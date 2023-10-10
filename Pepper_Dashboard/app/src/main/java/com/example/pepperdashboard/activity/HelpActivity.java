package com.example.pepperdashboard.activity;

import android.content.Intent;
import android.os.Bundle;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

import com.example.pepperdashboard.R;
import com.example.pepperdashboard.logic.RosTTS;

public class HelpActivity extends AppCompatActivity {
    private TextView helpText;
    private RosTTS tts;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.help_activity);

        helpText = findViewById(R.id.helpTextView);
        tts = new RosTTS(getIntent().getStringExtra("host"), 9090, this, getAssets());
        helpText.setText(tts.getText("help"));
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
